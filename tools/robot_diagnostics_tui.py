#!/usr/bin/env python3
"""
NonameRobot - Ultra-Lightweight TUI Diagnostics & Odometry Calibration Suite
Designed specifically for Raspberry Pi SSH / Terminal environments (Zero GUI overhead, Pure Curses)
Hospital Mobile Robot (X-Drive Omnidirectional) - ROS 2 Humble

Fitur Utama (Identik dengan versi GUI):
1. Multi-Tab Terminal UI:
   - [1] Dashboard Utama (Health Topik, TF Benchmark, Missed-Loop, Live Log Console)
   - [2] Kalibrasi & Uji Odometri (Live Pose, Test Maju/Geser/Rotasi, Kalkulator Skala, E-STOP)
   - [3+] Dynamic Topic Tabs (Tambah tab topik apa saja via tombol '+', live Hz, delay & payload)
2. Hemat Daya CPU (< 0.5% CPU di Raspberry Pi) menggunakan Python curses bawaan.
3. Kontrol Keyboard Penuh (Hotkey 1-9, Tab, Space E-STOP, +, s, r, q).
"""

import os
import sys
import time
import json
import math
import curses
import threading
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Log, Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_yaml

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False


# ==============================================================================
# 1. ROS 2 BACKEND NODE
# ==============================================================================
class TuiRosBackend(Node):
    def __init__(self):
        super().__init__('robot_diagnostics_tui_backend')

        self.ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        self.log_dir = os.path.join(self.ws_root, 'log', 'diagnostics_sessions')
        os.makedirs(self.log_dir, exist_ok=True)

        self.session_start_time = datetime.now()
        self.session_id = self.session_start_time.strftime("%Y%m%d_%H%M%S")

        # TF2 Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Sliding buffers
        self.scan_timestamps = deque(maxlen=40)
        self.odom_timestamps = deque(maxlen=60)
        self.cmd_vel_timestamps = deque(maxlen=40)
        self.raw_vel_timestamps = deque(maxlen=60)

        # Thread Lock & State
        self.lock = threading.Lock()
        self.latest_scan_stats = {'rate_hz': 0.0, 'delay_ms': 0.0, 'pts': 0, 'valid': 0, 'last_rx': 0.0}
        self.latest_odom_stats = {'rate_hz': 0.0, 'delay_ms': 0.0, 'x': 0.0, 'y': 0.0, 'theta': 0.0,
                                  'vx': 0.0, 'vy': 0.0, 'wz': 0.0, 'last_rx': 0.0}
        self.latest_raw_vel_stats = {'rate_hz': 0.0, 'delay_ms': 0.0, 'vx': 0.0, 'vy': 0.0, 'wz': 0.0, 'last_rx': 0.0}
        self.latest_cmd_vel_stats = {'rate_hz': 0.0, 'vx': 0.0, 'vy': 0.0, 'wz': 0.0, 'last_rx': 0.0}
        self.latest_tf_stats = {
            'odom_to_base': {'age_ms': None, 'lookup_duration_ms': 0.0, 'status': 'N/A'},
            'map_to_odom': {'age_ms': None, 'lookup_duration_ms': 0.0, 'status': 'N/A'},
            'map_to_base': {'age_ms': None, 'lookup_duration_ms': 0.0, 'status': 'N/A'}
        }

        # Missed Loops & Logs
        self.incidents = []
        self.total_missed_loops = 0
        self.recent_logs = deque(maxlen=50)
        self.metrics_history = []

        # Motion Test State
        self.motion_test_active = False
        self.motion_test_type = 'forward' # 'forward', 'strafe', 'rotate'
        self.motion_start_x = 0.0
        self.motion_start_y = 0.0
        self.motion_start_theta = 0.0
        self.motion_target_dist = 1.000
        self.motion_test_speed = 0.15
        self.last_test_distance_recorded = 0.0
        self.motion_progress_traveled = 0.0
        self.motion_status_msg = "Siap."

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reset_odom_pub = self.create_publisher(Empty, 'reset_odom', 10)

        # Dynamic Topics Monitor: {topic_name: {'sub': sub, 'type': type_str, 'info': dict}}
        self.dynamic_subs = {}

        # Standard Subscriptions
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_sensor)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_reliable)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, qos_reliable)
        self.create_subscription(TwistStamped, '/raw_odom_vel', self._raw_vel_cb, qos_reliable)
        self.create_subscription(Log, '/rosout', self._rosout_cb, qos_reliable)

        # Timers
        self.create_timer(0.1, self._tf_benchmark_cb)       # 10 Hz TF Benchmark
        self.create_timer(0.05, self._motion_test_step_cb)  # 20 Hz Motion Test Execution
        self.create_timer(1.0, self._metrics_snapshot_cb)   # 1 Hz Snapshot History

    def _scan_cb(self, msg: LaserScan):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delay_ms = (now_sec - stamp_sec) * 1000.0

        self.scan_timestamps.append(now_sec)
        rate_hz = 0.0
        if len(self.scan_timestamps) >= 2:
            dt = self.scan_timestamps[-1] - self.scan_timestamps[0]
            if dt > 0: rate_hz = (len(self.scan_timestamps) - 1) / dt

        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        with self.lock:
            self.latest_scan_stats = {
                'rate_hz': round(rate_hz, 2),
                'delay_ms': round(delay_ms, 2),
                'pts': len(msg.ranges),
                'valid': len(valid_ranges),
                'last_rx': now_sec
            }

    def _odom_cb(self, msg: Odometry):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delay_ms = (now_sec - stamp_sec) * 1000.0

        self.odom_timestamps.append(now_sec)
        rate_hz = 0.0
        if len(self.odom_timestamps) >= 2:
            dt = self.odom_timestamps[-1] - self.odom_timestamps[0]
            if dt > 0: rate_hz = (len(self.odom_timestamps) - 1) / dt

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        with self.lock:
            self.latest_odom_stats = {
                'rate_hz': round(rate_hz, 2),
                'delay_ms': round(delay_ms, 2),
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'theta': yaw,
                'vx': msg.twist.twist.linear.x,
                'vy': msg.twist.twist.linear.y,
                'wz': msg.twist.twist.angular.z,
                'last_rx': now_sec
            }

    def _raw_vel_cb(self, msg: TwistStamped):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delay_ms = (now_sec - stamp_sec) * 1000.0

        self.raw_vel_timestamps.append(now_sec)
        rate_hz = 0.0
        if len(self.raw_vel_timestamps) >= 2:
            dt = self.raw_vel_timestamps[-1] - self.raw_vel_timestamps[0]
            if dt > 0: rate_hz = (len(self.raw_vel_timestamps) - 1) / dt

        with self.lock:
            self.latest_raw_vel_stats = {
                'rate_hz': round(rate_hz, 2),
                'delay_ms': round(delay_ms, 2),
                'vx': msg.twist.linear.x,
                'vy': msg.twist.linear.y,
                'wz': msg.twist.angular.z,
                'last_rx': now_sec
            }

    def _cmd_vel_cb(self, msg: Twist):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_vel_timestamps.append(now_sec)
        rate_hz = 0.0
        if len(self.cmd_vel_timestamps) >= 2:
            dt = self.cmd_vel_timestamps[-1] - self.cmd_vel_timestamps[0]
            if dt > 0: rate_hz = (len(self.cmd_vel_timestamps) - 1) / dt

        with self.lock:
            self.latest_cmd_vel_stats = {
                'rate_hz': round(rate_hz, 2),
                'vx': msg.linear.x,
                'vy': msg.linear.y,
                'wz': msg.angular.z,
                'last_rx': now_sec
            }

    def _rosout_cb(self, msg: Log):
        if msg.level < 30: # Only WARN and ERROR
            return

        now_iso = datetime.now().strftime("%H:%M:%S")
        lvl = "WARN" if msg.level == 30 else "ERROR"
        entry = {'time': now_iso, 'node': msg.name, 'level': lvl, 'msg': msg.msg}

        with self.lock:
            self.recent_logs.append(entry)

        if "missed its desired rate" in msg.msg.lower() or "control loop missed" in msg.msg.lower():
            self._record_incident(msg, now_iso)

    def _record_incident(self, log_msg: Log, timestamp_str: str):
        with self.lock:
            self.total_missed_loops += 1
            causes = []
            tf_mb = self.latest_tf_stats.get('map_to_base', {})
            tf_lookup_ms = tf_mb.get('lookup_duration_ms', 0.0)
            tf_age_ms = tf_mb.get('age_ms', 0.0)

            if tf_lookup_ms is not None and tf_lookup_ms > 35.0:
                causes.append(f"TF lookup 'map->base_link' BLOCKING {tf_lookup_ms:.1f}ms")
            if tf_age_ms is not None and tf_age_ms > 120.0:
                causes.append(f"Transform 'map->base_link' basi ({tf_age_ms:.1f}ms)")
            
            scan_delay = self.latest_scan_stats.get('delay_ms', 0.0)
            if scan_delay > 100.0:
                causes.append(f"Delay /scan tinggi ({scan_delay:.1f}ms)")
            
            odom_rate = self.latest_odom_stats.get('rate_hz', 0.0)
            if odom_rate < 15.0 and odom_rate > 0.0:
                causes.append(f"Frekuensi /odom drop ({odom_rate:.1f}Hz)")

            if not causes:
                causes.append("Kemungkinan contention lock costmap / path planner.")

            self.incidents.append({
                'id': self.total_missed_loops,
                'time': timestamp_str,
                'node': log_msg.name,
                'msg': log_msg.msg,
                'causes': causes
            })

    def _tf_benchmark_cb(self):
        now_ros = self.get_clock().now()
        now_sec = now_ros.nanoseconds * 1e-9
        pairs = [
            ('odom_to_base', 'odom', 'base_link'),
            ('map_to_odom', 'map', 'odom'),
            ('map_to_base', 'map', 'base_link')
        ]

        for key, parent_f, child_f in pairs:
            t0 = time.perf_counter()
            try:
                tf = self.tf_buffer.lookup_transform(parent_f, child_f, rclpy.time.Time(),
                                                    timeout=rclpy.duration.Duration(seconds=0.025))
                dur_ms = (time.perf_counter() - t0) * 1000.0
                stamp_sec = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                age_ms = (now_sec - stamp_sec) * 1000.0

                with self.lock:
                    self.latest_tf_stats[key] = {
                        'age_ms': round(age_ms, 1),
                        'lookup_duration_ms': round(dur_ms, 2),
                        'status': 'OK' if dur_ms < 15.0 else 'SLOW'
                    }
            except Exception as ex:
                dur_ms = (time.perf_counter() - t0) * 1000.0
                with self.lock:
                    self.latest_tf_stats[key] = {
                        'age_ms': None,
                        'lookup_duration_ms': round(dur_ms, 2),
                        'status': type(ex).__name__[:8]
                    }

    def _metrics_snapshot_cb(self):
        now_sec = time.time()
        cpu_pct = psutil.cpu_percent() if HAS_PSUTIL else 0.0
        with self.lock:
            self.metrics_history.append({
                'time': round(now_sec, 2),
                'cpu_pct': cpu_pct,
                'scan': dict(self.latest_scan_stats),
                'odom': dict(self.latest_odom_stats),
                'tf': dict(self.latest_tf_stats),
                'missed_loops_total': self.total_missed_loops
            })
            if len(self.metrics_history) > 3600:
                self.metrics_history.pop(0)

    # ------------------ MOTION TEST & JOG ------------------
    def start_motion_test(self, test_type: str, target_val: float, speed: float = 0.15):
        with self.lock:
            self.motion_test_type = test_type
            self.motion_target_dist = abs(target_val)
            self.motion_test_speed = abs(speed)
            self.motion_start_x = self.latest_odom_stats['x']
            self.motion_start_y = self.latest_odom_stats['y']
            self.motion_start_theta = self.latest_odom_stats['theta']
            self.motion_progress_traveled = 0.0
            self.motion_status_msg = f"Menjalankan uji {test_type.upper()} ({target_val:.2f})..."
            self.motion_test_active = True

    def stop_motion_test(self):
        with self.lock:
            self.motion_test_active = False
            self.motion_status_msg = "Uji dihentikan."
        self.send_cmd_vel(0.0, 0.0, 0.0)

    def _motion_test_step_cb(self):
        if not self.motion_test_active:
            return

        with self.lock:
            curr_x = self.latest_odom_stats['x']
            curr_y = self.latest_odom_stats['y']
            curr_th = self.latest_odom_stats['theta']
            test_type = self.motion_test_type
            target = self.motion_target_dist
            spd = self.motion_test_speed
            start_x = self.motion_start_x
            start_y = self.motion_start_y
            start_th = self.motion_start_theta

        dx = curr_x - start_x
        dy = curr_y - start_y
        dtheta = abs(curr_th - start_th)
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

        if test_type == 'forward':
            traveled = math.sqrt(dx * dx + dy * dy)
            self.motion_progress_traveled = traveled
            if (target - traveled) <= 0.01:
                self.stop_motion_test()
                self.last_test_distance_recorded = traveled
                self.motion_status_msg = f"SELESAI! Jarak Odom: {traveled:.3f} m"
                return
            self.send_cmd_vel(spd, 0.0, 0.0)
            self.motion_status_msg = f"Maju: {traveled:.3f} / {target:.3f} m"

        elif test_type == 'strafe':
            traveled = math.sqrt(dx * dx + dy * dy)
            self.motion_progress_traveled = traveled
            if (target - traveled) <= 0.01:
                self.stop_motion_test()
                self.last_test_distance_recorded = traveled
                self.motion_status_msg = f"SELESAI! Geser Odom: {traveled:.3f} m"
                return
            self.send_cmd_vel(0.0, spd, 0.0)
            self.motion_status_msg = f"Geser Y: {traveled:.3f} / {target:.3f} m"

        elif test_type == 'rotate':
            traveled_deg = math.degrees(dtheta)
            self.motion_progress_traveled = traveled_deg
            if (target - traveled_deg) <= 1.0:
                self.stop_motion_test()
                self.last_test_distance_recorded = traveled_deg
                self.motion_status_msg = f"SELESAI! Rotasi Odom: {traveled_deg:.1f}°"
                return
            self.send_cmd_vel(0.0, 0.0, spd)
            self.motion_status_msg = f"Rotasi: {traveled_deg:.1f}° / {target:.1f}°"

    def send_cmd_vel(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def trigger_reset_odom(self):
        self.reset_odom_pub.publish(Empty())

    # ------------------ DYNAMIC TOPIC MONITORING ------------------
    def subscribe_dynamic_topic(self, topic_name: str, type_str: str):
        if topic_name in self.dynamic_subs:
            return True

        try:
            msg_cls = get_message(type_str)
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)

            sub_info = {
                'type': type_str,
                'timestamps': deque(maxlen=40),
                'total_count': 0,
                'last_delay_ms': None,
                'current_rate_hz': 0.0,
                'last_msg_yaml': '',
                'is_paused': False
            }

            def internal_cb(msg):
                if sub_info['is_paused']:
                    return
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                sub_info['total_count'] += 1
                sub_info['timestamps'].append(now_sec)

                if len(sub_info['timestamps']) >= 2:
                    dt = sub_info['timestamps'][-1] - sub_info['timestamps'][0]
                    if dt > 0: sub_info['current_rate_hz'] = (len(sub_info['timestamps']) - 1) / dt

                if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                    s_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    sub_info['last_delay_ms'] = (now_sec - s_sec) * 1000.0

                try:
                    yaml_text = message_to_yaml(msg)
                except Exception:
                    yaml_text = str(msg)
                sub_info['last_msg_yaml'] = yaml_text

            sub = self.create_subscription(msg_cls, topic_name, internal_cb, qos)
            self.dynamic_subs[topic_name] = {'sub': sub, 'info': sub_info}
            return True
        except Exception as e:
            self.get_logger().error(f"Gagal subscribe topik TUI '{topic_name}': {e}")
            return False

    def unsubscribe_dynamic_topic(self, topic_name: str):
        if topic_name in self.dynamic_subs:
            sub_obj = self.dynamic_subs[topic_name]['sub']
            self.destroy_subscription(sub_obj)
            del self.dynamic_subs[topic_name]

    # ------------------ RUNTIME PARAMETER CLIENT ------------------
    def set_odometry_scale_param(self, param_name: str, new_val: float):
        cli = self.create_client(SetParameters, '/odometry_node/set_parameters')
        if not cli.wait_for_service(timeout_sec=1.0):
            return False, "Service parameter odometry_node tidak tersedia."

        req = SetParameters.Request()
        p = Parameter()
        p.name = param_name
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = float(new_val)
        req.parameters = [p]

        future = cli.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0 < 1.0):
            time.sleep(0.05)

        if future.done():
            res = future.result()
            if res and res.results and res.results[0].successful:
                return True, f"Berhasil set {param_name} = {new_val:.4f}"
            else:
                return False, f"Gagal: {res.results[0].reason if res.results else 'Unknown'}"
        return False, "Timeout request parameter."


# ==============================================================================
# 2. TUI CURSES INTERFACE
# ==============================================================================
class DiagnosticsTuiApp:
    def __init__(self, backend: TuiRosBackend):
        self.backend = backend
        self.active_tab_idx = 0 # 0: Dashboard, 1: Calibration, 2+: Dynamic Topics
        self.dynamic_tab_topics = [] # List of topic names added as tabs

        # Calibration form states
        self.calib_axis = 'linear_scale_x' # 'linear_scale_x', 'linear_scale_y', 'angular_scale_z'
        self.calib_current_scale = 1.050
        self.calib_odom_dist = 0.872
        self.calib_actual_dist = 1.000
        self.calib_calculated_scale = None
        self.tare_x = 0.0
        self.tare_y = 0.0

        # Status notification banner in footer
        self.status_banner = "Siap. Tekan [1-9] ganti Tab, [h] Bantuan."
        self.status_banner_time = time.time()

    def set_status(self, msg: str):
        self.status_banner = msg
        self.status_banner_time = time.time()

    def run(self, stdscr):
        curses.curs_set(0) # Hide blinking cursor
        curses.use_default_colors()
        stdscr.nodelay(True) # Non-blocking input
        stdscr.timeout(100) # 100ms refresh loop (10 Hz)

        # Color Pairs Init
        if curses.has_colors():
            curses.start_color()
            curses.init_pair(1, curses.COLOR_CYAN, -1)     # Cyan header
            curses.init_pair(2, curses.COLOR_GREEN, -1)    # Green status OK
            curses.init_pair(3, curses.COLOR_YELLOW, -1)   # Yellow warning
            curses.init_pair(4, curses.COLOR_RED, -1)      # Red error
            curses.init_pair(5, curses.COLOR_BLACK, curses.COLOR_CYAN) # Active tab
            curses.init_pair(6, curses.COLOR_WHITE, curses.COLOR_BLUE) # Title banner

        while True:
            # Handle Key Input
            try:
                ch = stdscr.getch()
            except Exception:
                ch = -1

            if ch != -1:
                if ch == ord('q') or ch == ord('Q'):
                    break
                self._handle_input(ch, stdscr)

            # Redraw Terminal Screen
            stdscr.erase()
            h, w = stdscr.getmaxyx()

            if h < 20 or w < 75:
                stdscr.addstr(0, 0, f"Ukuran terminal terlalu kecil ({w}x{h}). Minimal 80x24.", curses.color_pair(3))
            else:
                self._render_header(stdscr, w)
                self._render_tabs_bar(stdscr, w)

                if self.active_tab_idx == 0:
                    self._render_dashboard_tab(stdscr, h, w)
                elif self.active_tab_idx == 1:
                    self._render_calibration_tab(stdscr, h, w)
                else:
                    dyn_idx = self.active_tab_idx - 2
                    if dyn_idx < len(self.dynamic_tab_topics):
                        self._render_dynamic_topic_tab(stdscr, h, w, self.dynamic_tab_topics[dyn_idx])

                self._render_footer(stdscr, h, w)

            stdscr.refresh()

    # ==================== INPUT HANDLER ====================
    def _handle_input(self, ch: int, stdscr):
        total_tabs = 2 + len(self.dynamic_tab_topics)

        # Tab Switching hotkeys: 1-9
        if ord('1') <= ch <= ord('9'):
            target_idx = ch - ord('1')
            if target_idx < total_tabs:
                self.active_tab_idx = target_idx

        # Left / Right Arrow or Tab
        elif ch == curses.KEY_RIGHT or ch == 9: # Tab key
            self.active_tab_idx = (self.active_tab_idx + 1) % total_tabs
        elif ch == curses.KEY_LEFT or ch == curses.KEY_BTAB:
            self.active_tab_idx = (self.active_tab_idx - 1 + total_tabs) % total_tabs

        # Global Hotkey: Save Session [s]
        elif ch == ord('s') or ch == ord('S'):
            self._save_session_files()

        # Global Hotkey: Emergency Stop [SPACE]
        elif ch == ord(' '):
            self.backend.stop_motion_test()
            self.backend.send_cmd_vel(0.0, 0.0, 0.0)
            self.set_status("🛑 EMERGENCY STOP DIAKTIFKAN!")

        # Global Hotkey: Reset Odometry Node [r]
        elif ch == ord('r') or ch == ord('R'):
            self.backend.trigger_reset_odom()
            self.tare_x = 0.0
            self.tare_y = 0.0
            self.set_status("🔄 Permintaan Reset Odometri (/reset_odom) dikirim!")

        # Global Hotkey: Tambah Topik Baru [+] or [a]
        elif ch == ord('+') or ch == ord('a') or ch == ord('A'):
            self._prompt_add_topic(stdscr)

        # Close Dynamic Tab [x]
        elif (ch == ord('x') or ch == ord('X')) and self.active_tab_idx >= 2:
            dyn_idx = self.active_tab_idx - 2
            t_name = self.dynamic_tab_topics.pop(dyn_idx)
            self.backend.unsubscribe_dynamic_topic(t_name)
            self.active_tab_idx = min(self.active_tab_idx, len(self.dynamic_tab_topics) + 1)
            self.set_status(f"Tab topik '{t_name}' ditutup.")

        # Tab 2 (Calibration) Specific Controls
        elif self.active_tab_idx == 1:
            self._handle_calibration_input(ch)

        # Dynamic Tab Specific Controls (Pause [p], Clear [c])
        elif self.active_tab_idx >= 2:
            dyn_idx = self.active_tab_idx - 2
            t_name = self.dynamic_tab_topics[dyn_idx]
            sub_info = self.backend.dynamic_subs.get(t_name, {}).get('info')
            if sub_info:
                if ch == ord('p') or ch == ord('P'):
                    sub_info['is_paused'] = not sub_info.get('is_paused', False)
                    self.set_status("PAUSED" if sub_info['is_paused'] else "RESUMED")
                elif ch == ord('c') or ch == ord('C'):
                    sub_info['last_msg_yaml'] = ""
                    self.set_status("Cleared view.")

    def _handle_calibration_input(self, ch: int):
        # Software Tare [z]
        if ch == ord('z') or ch == ord('Z'):
            with self.backend.lock:
                self.tare_x = self.backend.latest_odom_stats['x']
                self.tare_y = self.backend.latest_odom_stats['y']
            self.set_status(f"Software Tare set ke X={self.tare_x:.3f}, Y={self.tare_y:.3f}")

        # Run Motion Test: Maju 1.0m [m]
        elif ch == ord('m') or ch == ord('M'):
            self.backend.start_motion_test('forward', 1.000, 0.15)
            self.set_status("Menjalankan Uji Maju 1.000 meter...")

        # Run Strafe Test: Geser 0.5m [g]
        elif ch == ord('g') or ch == ord('G'):
            self.backend.start_motion_test('strafe', 0.500, 0.15)
            self.set_status("Menjalankan Uji Geser 0.500 meter...")

        # Run Rotate Test: Putar 360 deg [p]
        elif ch == ord('p') or ch == ord('P'):
            self.backend.start_motion_test('rotate', 360.0, 0.50)
            self.set_status("Menjalankan Uji Rotasi 360 derajat...")

        # Select Axis: [x], [y], [w]
        elif ch == ord('x'): self.calib_axis = 'linear_scale_x'; self.set_status("Sumbu: linear_scale_x")
        elif ch == ord('y'): self.calib_axis = 'linear_scale_y'; self.set_status("Sumbu: linear_scale_y")
        elif ch == ord('w'): self.calib_axis = 'angular_scale_z'; self.set_status("Sumbu: angular_scale_z")

        # Copy Last Test Dist [u]
        elif ch == ord('u') or ch == ord('U'):
            rec = self.backend.last_test_distance_recorded
            if rec > 0.0:
                self.calib_odom_dist = rec
                self.set_status(f"Jarak odometri diset dari hasil uji: {rec:.3f} m")
            else:
                self.set_status("Belum ada hasil uji gerak.")

        # Adjust Target Actual Dist [+] / [-]
        elif ch == ord('='):
            self.calib_actual_dist = round(self.calib_actual_dist + 0.05, 3)
        elif ch == ord('-'):
            self.calib_actual_dist = max(0.1, round(self.calib_actual_dist - 0.05, 3))

        # Calculate Scale [c]
        elif ch == ord('c') or ch == ord('C'):
            if self.calib_odom_dist > 0:
                self.calib_calculated_scale = self.calib_current_scale * (self.calib_actual_dist / self.calib_odom_dist)
                delta = ((self.calib_calculated_scale - self.calib_current_scale) / self.calib_current_scale) * 100.0
                self.set_status(f"Skala baru dihitung: {self.calib_calculated_scale:.4f} ({delta:+.1f}%)")

        # Apply Runtime [t]
        elif ch == ord('t') or ch == ord('T'):
            if self.calib_calculated_scale is None:
                self.calib_calculated_scale = self.calib_current_scale * (self.calib_actual_dist / self.calib_odom_dist)
            ok, msg = self.backend.set_odometry_scale_param(self.calib_axis, self.calib_calculated_scale)
            if ok:
                self.calib_current_scale = self.calib_calculated_scale
                self.set_status(f"✅ {msg}")
            else:
                self.set_status(f"⚠️ {msg}")

        # Save to YAML [k]
        elif ch == ord('k') or ch == ord('K'):
            self._save_scale_to_files()

    # ==================== RENDERERS ====================
    def _render_header(self, stdscr, w):
        cpu_pct = psutil.cpu_percent() if HAS_PSUTIL else 0.0
        mem = psutil.virtual_memory() if HAS_PSUTIL else None
        mem_str = f"{mem.used / 1e9:.1f}/{mem.total / 1e9:.1f}GB" if mem else "N/A"
        header_text = f" 🤖 NONAME ROBOT TUI DIAGNOSTICS [Sesi: {self.backend.session_id}] "
        stats_text = f"CPU: {cpu_pct:4.1f}% | RAM: {mem_str} "

        stdscr.attron(curses.color_pair(6) | curses.A_BOLD)
        stdscr.addstr(0, 0, " " * w)
        stdscr.addstr(0, 1, header_text)
        if len(header_text) + len(stats_text) < w:
            stdscr.addstr(0, w - len(stats_text) - 1, stats_text)
        stdscr.attroff(curses.color_pair(6) | curses.A_BOLD)

    def _render_tabs_bar(self, stdscr, w):
        tabs = ["[1] 📊 Dashboard", "[2] 🎯 Kalibrasi Odom"]
        for idx, t_name in enumerate(self.dynamic_tab_topics):
            tabs.append(f"[{idx+3}] {t_name}")
        tabs.append("[+] Tambah")

        x = 1
        for idx, title in enumerate(tabs):
            is_active = (idx == self.active_tab_idx) or (idx == len(tabs) - 1 and False)
            style = curses.color_pair(5) | curses.A_BOLD if is_active else curses.A_NORMAL
            stdscr.addstr(1, x, f" {title} ", style)
            x += len(title) + 3
            if x >= w - 10:
                break
        stdscr.addstr(2, 0, "─" * w)

    # ------------------ TAB 1: DASHBOARD UTAMA ------------------
    def _render_dashboard_tab(self, stdscr, h, w):
        with self.backend.lock:
            scan = dict(self.backend.latest_scan_stats)
            odom = dict(self.backend.latest_odom_stats)
            cmd = dict(self.backend.latest_cmd_vel_stats)
            raw = dict(self.backend.latest_raw_vel_stats)
            tfs = dict(self.backend.latest_tf_stats)
            missed = self.backend.total_missed_loops
            incidents = list(self.backend.incidents)
            logs = list(self.backend.recent_logs)

        # 1. Health & Frekuensi Topik
        stdscr.addstr(3, 1, "1. KESEGARAN & FREKUENSI TOPIK UTAMA", curses.A_BOLD | curses.color_pair(1))
        header_fmt = "  {:<24} {:<12} {:<14} {:<10} {:<16}"
        stdscr.addstr(4, 1, header_fmt.format("Topik", "Frekuensi", "Delay/Latensi", "Status", "Info Singkat"), curses.A_UNDERLINE)

        def row(y, name, rate, delay, status, info):
            s_color = curses.color_pair(2) if status in ("OK", "ACTIVE") else curses.color_pair(3)
            stdscr.addstr(y, 1, f"  {name:<24} {rate:<12} {delay:<14} ")
            stdscr.addstr(status, s_color | curses.A_BOLD)
            stdscr.addstr(f"  {info[:w-60]}")

        row(5, "/scan (LiDAR A1)", f"{scan['rate_hz']:.1f} Hz", f"{scan['delay_ms']:.1f} ms",
            "OK" if scan['rate_hz'] > 5.0 else "SLOW", f"Pts: {scan['valid']}/{scan['pts']}")
        row(6, "/odom (Odometri Roda)", f"{odom['rate_hz']:.1f} Hz", f"{odom['delay_ms']:.1f} ms",
            "OK" if odom['rate_hz'] > 15.0 else "SLOW", f"Vx:{odom['vx']:.2f} Vy:{odom['vy']:.2f} Wz:{odom['wz']:.2f}")
        row(7, "/cmd_vel (Controller)", f"{cmd['rate_hz']:.1f} Hz", "--",
            "ACTIVE" if cmd['rate_hz'] > 0 else "IDLE", f"Vx:{cmd['vx']:.2f} Vy:{cmd['vy']:.2f} Wz:{cmd['wz']:.2f}")
        row(8, "/raw_odom_vel (Low-level)", f"{raw['rate_hz']:.1f} Hz", f"{raw['delay_ms']:.1f} ms",
            "OK" if raw['rate_hz'] > 15.0 else "WAIT", f"Vx:{raw['vx']:.2f} Vy:{raw['vy']:.2f} Wz:{raw['wz']:.2f}")

        stdscr.addstr(9, 0, "─" * w)

        # 2. TF Latency & Benchmark
        stdscr.addstr(10, 1, "2. LATENSI TF (MAP -> ODOM -> BASE_LINK)", curses.A_BOLD | curses.color_pair(1))
        ob = tfs.get('odom_to_base', {})
        mo = tfs.get('map_to_odom', {})
        mb = tfs.get('map_to_base', {})

        def tf_str(d):
            age = f"{d.get('age_ms')}ms" if d.get('age_ms') is not None else "N/A"
            dur = f"{d.get('lookup_duration_ms', 0):.1f}ms"
            st = d.get('status', 'N/A')
            return f"Age: {age:<9} Dur: {dur:<7} [{st}]"

        stdscr.addstr(11, 2, f"• odom -> base_link : {tf_str(ob)}")
        stdscr.addstr(12, 2, f"• map  -> odom      : {tf_str(mo)}")
        stdscr.addstr(13, 2, f"• map  -> base_link : {tf_str(mb)}")

        stdscr.addstr(14, 0, "─" * w)

        # 3. Missed Loops
        m_color = curses.color_pair(2) if missed == 0 else curses.color_pair(4)
        stdscr.addstr(15, 1, "3. MISSED-LOOP CONTROLLER SERVER (10Hz): ", curses.A_BOLD | curses.color_pair(1))
        stdscr.addstr(f"{missed} Insiden Terdeteksi", m_color | curses.A_BOLD)

        if incidents:
            last = incidents[-1]
            cause_preview = " | ".join(last.get('causes', []))
            stdscr.addstr(16, 2, f"Terakhir #{last['id']} [{last['time']}] oleh {last['node']}: {cause_preview[:w-35]}", curses.color_pair(3))
        else:
            stdscr.addstr(16, 2, "✅ Normal - Belum ada missed-loop terdeteksi pada controller 10Hz.", curses.color_pair(2))

        stdscr.addstr(17, 0, "─" * w)

        # 4. Live Log Console
        stdscr.addstr(18, 1, "4. LIVE ROS 2 WARNING & ERROR LOG CONSOLE (/rosout)", curses.A_BOLD | curses.color_pair(1))
        y = 19
        max_log_lines = max(1, h - y - 3)
        shown_logs = logs[-max_log_lines:]
        for entry in shown_logs:
            if y >= h - 2: break
            l_color = curses.color_pair(4) if entry['level'] == 'ERROR' else curses.color_pair(3)
            log_line = f"[{entry['time']}] [{entry['level']}] [{entry['node']}]: {entry['msg']}"
            stdscr.addstr(y, 1, log_line[:w-3], l_color)
            y += 1

    # ------------------ TAB 2: KALIBRASI & UJI ODOMETRI ------------------
    def _render_calibration_tab(self, stdscr, h, w):
        with self.backend.lock:
            odom = dict(self.backend.latest_odom_stats)
            status_msg = self.backend.motion_status_msg
            rec_dist = self.backend.last_test_distance_recorded
            traveled = self.backend.motion_progress_traveled
            target = self.backend.motion_target_dist
            is_active = self.backend.motion_test_active

        # 1. Live Odometry State
        stdscr.addstr(3, 1, "1. MONITOR ODOMETRI REAL-TIME", curses.A_BOLD | curses.color_pair(1))
        dx = odom['x'] - self.tare_x
        dy = odom['y'] - self.tare_y
        dist_tare = math.sqrt(dx * dx + dy * dy)
        th_deg = math.degrees(odom['theta'])

        stdscr.addstr(4, 2, f"Posisi X     : {odom['x']:+8.3f} m      |  Kecepatan Vx : {odom['vx']:+6.2f} m/s")
        stdscr.addstr(5, 2, f"Posisi Y     : {odom['y']:+8.3f} m      |  Kecepatan Vy : {odom['vy']:+6.2f} m/s")
        stdscr.addstr(6, 2, f"Yaw (Theta)  : {th_deg:+6.1f}° ({odom['theta']:+5.2f} rad) |  Kecepatan Wz : {odom['wz']:+6.2f} rad/s")
        stdscr.addstr(7, 2, f"Jarak Relatif: {dist_tare:8.3f} m (Sejak Tare)")
        stdscr.addstr(8, 2, "Aksi Cepat   : [z] Set Tare Zero (GUI)  |  [r] Reset Node Odometri (/reset_odom)", curses.A_BOLD | curses.color_pair(3))

        stdscr.addstr(9, 0, "─" * w)

        # 2. Automated Motion Test
        stdscr.addstr(10, 1, "2. UJI GERAK OTOMATIS (MOTION DISTANCE TEST)", curses.A_BOLD | curses.color_pair(1))
        stdscr.addstr(11, 2, "Perintah Uji : [m] Maju 1.0m (X)  |  [g] Geser 0.5m (Y)  |  [p] Putar 360° (Yaw)", curses.A_BOLD)
        stdscr.addstr(12, 2, f"Status Gerak : {status_msg}", curses.color_pair(2) if "SELESAI" in status_msg else curses.color_pair(3))

        # Progress bar
        bar_w = min(40, w - 25)
        pct = min(1.0, traveled / target) if target > 0 else 0.0
        filled = int(bar_w * pct)
        bar_str = "[" + ("█" * filled) + ("░" * (bar_w - filled)) + f"] {int(pct*100)}%"
        stdscr.addstr(13, 2, f"Progress     : {bar_str}")
        stdscr.addstr(14, 2, "KONTROL STOP : [SPACE] EMERGENCY STOP (E-STOP)!", curses.A_BOLD | curses.color_pair(4))

        stdscr.addstr(15, 0, "─" * w)

        # 3. Scale Calibration Calculator
        stdscr.addstr(16, 1, "3. KALKULATOR KALIBRASI SKALA ODOMETRI", curses.A_BOLD | curses.color_pair(1))
        stdscr.addstr(17, 2, f"Sumbu Aktif  : [{self.calib_axis}]  ([x] linear_x  [y] linear_y  [w] angular_z)", curses.A_BOLD)
        stdscr.addstr(18, 2, f"Jarak Odom   : {self.calib_odom_dist:6.3f} m  ([u] Ambil dari Hasil Uji Terakhir: {rec_dist:.3f} m)")
        stdscr.addstr(19, 2, f"Jarak Aktual : {self.calib_actual_dist:6.3f} m  ([+] Tambah 5cm  [-] Kurang 5cm)")
        stdscr.addstr(20, 2, f"Skala Lama   : {self.calib_current_scale:.4f}")

        # Calculation result
        if self.calib_calculated_scale is not None:
            delta = ((self.calib_calculated_scale - self.calib_current_scale) / self.calib_current_scale) * 100.0
            stdscr.addstr(21, 2, f"👉 REKOMENDASI SKALA BARU: {self.calib_calculated_scale:.4f} ({delta:+.2f}%)", curses.A_BOLD | curses.color_pair(2))
        else:
            stdscr.addstr(21, 2, "👉 Tekan [c] untuk Menghitung Skala Baru.", curses.A_ITALIC)

        stdscr.addstr(22, 2, "Terapkan     : [t] Terapkan Runtime ke Node  |  [k] Simpan ke nav2_params_paper.yaml", curses.A_BOLD | curses.color_pair(3))

    # ------------------ TAB 3+: DYNAMIC TOPIC TAB ------------------
    def _render_dynamic_topic_tab(self, stdscr, h, w, topic_name: str):
        sub_data = self.backend.dynamic_subs.get(topic_name, {})
        info = sub_data.get('info', {})
        t_type = info.get('type', 'Unknown')
        rate = info.get('current_rate_hz', 0.0)
        delay = info.get('last_delay_ms')
        count = info.get('total_count', 0)
        paused = info.get('is_paused', False)
        yaml_payload = info.get('last_msg_yaml', '(Menunggu data pesan pertama...)')

        delay_str = f"{delay:.1f} ms" if delay is not None else "N/A"
        p_str = "[PAUSED]" if paused else "[STREAMING]"

        stdscr.addstr(3, 1, f"Topik: {topic_name}  |  Tipe: {t_type}  |  {p_str}", curses.A_BOLD | curses.color_pair(1))
        stdscr.addstr(4, 1, f"Freq: {rate:.1f} Hz  |  Delay: {delay_str}  |  Total Pesan: {count}  |  [p] Pause  [c] Clear  [x] Tutup Tab", curses.A_UNDERLINE)

        # Render message YAML
        lines = yaml_payload.split('\n')
        max_lines = max(1, h - 7)
        for i, line in enumerate(lines[:max_lines]):
            stdscr.addstr(6 + i, 2, line[:w-5])

    # ------------------ FOOTER ------------------
    def _render_footer(self, stdscr, h, w):
        stdscr.addstr(h - 2, 0, "─" * w)
        # Check banner expiry (show for 4 seconds)
        if time.time() - self.status_banner_time < 5.0:
            stdscr.addstr(h - 1, 1, self.status_banner[:w-2], curses.A_BOLD | curses.color_pair(3))
        else:
            hint = "[1-9] Ganti Tab | [Left/Right] Navigasi | [+] Tambah Topik | [s] Simpan Sesi | [q] Keluar"
            stdscr.addstr(h - 1, 1, hint[:w-2], curses.A_DIM)

    # ==================== DIALOG / PROMPTS ====================
    def _prompt_add_topic(self, stdscr):
        curses.echo()
        curses.curs_set(1)
        h, w = stdscr.getmaxyx()

        # Fetch available topics
        available = []
        try:
            available = self.backend.get_topic_names_and_types()
        except Exception:
            pass

        # Simple input popup box
        box_w = min(70, w - 4)
        box_h = 14
        start_y = max(2, (h - box_h) // 2)
        start_x = max(2, (w - box_w) // 2)

        win = curses.newwin(box_h, box_w, start_y, start_x)
        win.box()
        win.addstr(1, 2, "➕ TAMBAH MONITOR TOPIK ROS 2", curses.A_BOLD)
        win.addstr(2, 2, "Topik aktif terdeteksi:", curses.A_UNDERLINE)

        for i, (t_name, t_types) in enumerate(available[:6]):
            win.addstr(3 + i, 3, f"• {t_name} ({t_types[0] if t_types else '?'})"[:box_w-6])

        win.addstr(10, 2, "Ketik nama topik: ")
        win.refresh()

        try:
            input_bytes = win.getstr(10, 20, box_w - 24)
            chosen_topic = input_bytes.decode('utf-8').strip()
        except Exception:
            chosen_topic = ""

        curses.noecho()
        curses.curs_set(0)

        if not chosen_topic:
            self.set_status("Batal menambah topik.")
            return

        # Find type
        matched_type = None
        for t_name, t_types in available:
            if t_name == chosen_topic and t_types:
                matched_type = t_types[0]
                break

        if not matched_type:
            # Fallback common types
            if "odom" in chosen_topic: matched_type = "nav_msgs/msg/Odometry"
            elif "scan" in chosen_topic: matched_type = "sensor_msgs/msg/LaserScan"
            elif "vel" in chosen_topic: matched_type = "geometry_msgs/msg/Twist"
            else: matched_type = "std_msgs/msg/String"

        if chosen_topic not in self.dynamic_tab_topics:
            ok = self.backend.subscribe_dynamic_topic(chosen_topic, matched_type)
            if ok:
                self.dynamic_tab_topics.append(chosen_topic)
                self.active_tab_idx = 1 + len(self.dynamic_tab_topics)
                self.set_status(f"✅ Tab baru dibuka: {chosen_topic}")
            else:
                self.set_status(f"❌ Gagal subscribe ke {chosen_topic}")
        else:
            self.set_status(f"Tab untuk {chosen_topic} sudah ada.")

    # ==================== SAVE SESSION ====================
    def _save_session_files(self):
        sess_id = self.backend.session_id
        json_file = os.path.join(self.backend.log_dir, f"session_{sess_id}.json")
        md_file = os.path.join(self.backend.log_dir, f"session_{sess_id}_report.md")

        with self.backend.lock:
            incidents_copy = list(self.backend.incidents)
            history_copy = list(self.backend.metrics_history)
            scan_stats = dict(self.backend.latest_scan_stats)
            odom_stats = dict(self.backend.latest_odom_stats)
            total_missed = self.backend.total_missed_loops

        now = datetime.now()
        duration = (now - self.backend.session_start_time).total_seconds()

        export_data = {
            'session_id': sess_id,
            'start_time': self.backend.session_start_time.isoformat(),
            'end_time': now.isoformat(),
            'duration_seconds': duration,
            'summary': {
                'total_missed_loops': total_missed,
                'scan_rate_hz': scan_stats.get('rate_hz', 0.0),
                'scan_delay_ms': scan_stats.get('delay_ms', 0.0),
                'odom_rate_hz': odom_stats.get('rate_hz', 0.0),
            },
            'incidents': incidents_copy,
            'time_series_metrics': history_copy
        }

        try:
            with open(json_file, 'w') as f:
                json.dump(export_data, f, indent=2)

            with open(md_file, 'w') as f:
                f.write(f"# Laporan Diagnostik Nav2 (TUI) - Sesi {sess_id}\n\n")
                f.write(f"- **Waktu**: {self.backend.session_start_time} s/d {now} ({duration:.1f} detik)\n")
                f.write(f"- **Total Missed Loops**: **{total_missed} kali**\n")
                f.write(f"- **Frekuensi LiDAR (/scan)**: {scan_stats.get('rate_hz', 0.0)} Hz\n")
                f.write(f"- **Frekuensi Odometri (/odom)**: {odom_stats.get('rate_hz', 0.0)} Hz\n\n")
                f.write("## Rekaman Insiden:\n")
                if not incidents_copy:
                    f.write("✅ Tidak ada insiden missed-loop selama sesi ini.\n")
                else:
                    for inc in incidents_copy:
                        f.write(f"### Insiden #{inc['id']} - {inc['time']}\n")
                        f.write(f"- Pesan: `{inc['msg']}`\n")
                        f.write(f"- Penyebab: {', '.join(inc.get('causes', []))}\n\n")

            self.set_status(f"💾 Disimpan ke: session_{sess_id}.json & .md")
        except Exception as e:
            self.set_status(f"❌ Gagal menyimpan sesi: {e}")

    def _save_scale_to_files(self):
        if self.calib_calculated_scale is None:
            self.set_status("⚠️ Hitung skala baru terlebih dahulu dengan [c].")
            return

        axis = self.calib_axis
        new_scale = self.calib_calculated_scale
        py_path = os.path.join(self.backend.ws_root, 'src', 'robot_base', 'robot_base', 'odometry_node.py')

        try:
            if os.path.exists(py_path):
                with open(py_path, 'r') as f:
                    content = f.read()

                import re
                pattern = rf"self\.declare_parameter\('{axis}',\s*[\d\.]+\)"
                repl = f"self.declare_parameter('{axis}', {new_scale:.4f})"
                new_content = re.sub(pattern, repl, content)

                with open(py_path, 'w') as f:
                    f.write(new_content)

                self.set_status(f"💾 Tersimpan permanen ke odometry_node.py ({axis} = {new_scale:.4f})")
            else:
                self.set_status("File odometry_node.py tidak ditemukan.")
        except Exception as e:
            self.set_status(f"❌ Error menyimpan file: {e}")


# ==============================================================================
# 3. MAIN ENTRYPOINT
# ==============================================================================
def main():
    rclpy.init(args=sys.argv)
    ros_backend = TuiRosBackend()

    # Spin ROS 2 in background thread
    t = threading.Thread(target=rclpy.spin, args=(ros_backend,), daemon=True)
    t.start()

    app = DiagnosticsTuiApp(ros_backend)
    try:
        curses.wrapper(app.run)
    except KeyboardInterrupt:
        pass
    finally:
        # Save session before exit
        app._save_session_files()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
