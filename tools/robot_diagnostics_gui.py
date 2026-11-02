#!/usr/bin/env python3
"""
NonameRobot - Multi-Topic Diagnostics, Flight Recorder & Odometry Calibration GUI
Hospital Mobile Robot (X-Drive Omnidirectional) - ROS 2 Humble

Fitur Utama:
1. Dashboard Utama:
   - Status & Frekuensi Topik Kritis (/scan, /odom, /cmd_vel, /raw_odom_vel).
   - TF Chain Latency & Blocking Benchmark (map -> odom -> base_link).
   - Missed-Loop Incident Recorder (Controller Server 10Hz monitor & cause-effect analysis).
   - Live Warning & Error Console (/rosout).
   - Status Host CPU per-core & RAM.
2. Dynamic Multi-Tab Topic Monitor:
   - Pengguna dapat membuka tab sebanyak mungkin untuk topik apapun yang aktif di ROS 2.
   - Deteksi otomatis tipe pesan, penghitung frekuensi (Hz), delay timestamp (ms), dan live YAML inspector.
3. Window & Tab Kalibrasi / Uji Odometri:
   - Live pose (X, Y, Theta) dan kecepatan (Vx, Vy, Wz).
   - Tombol Zero / Tare dan Reset Odometri (/reset_odom).
   - Uji Gerak Otomatis (Uji Maju 1.0m, Uji Geser Y, Uji Putar 360°) dengan tombol E-STOP.
   - Manual Jog Controller.
   - Kalkulator Kalibrasi Skala (linear_scale_x, linear_scale_y, angular_scale_z)
     dengan tombol "Terapkan Runtime ke Node" dan "Simpan ke nav2_params_paper.yaml".
4. Perekam Sesi Diagnostik (Export JSON & Markdown Report).
"""

import os
import sys
import time
import json
import math
import threading
from datetime import datetime
from collections import deque

# PyQt5 Imports
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPalette, QIcon
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget, QVBoxLayout,
    QHBoxLayout, QGridLayout, QLabel, QPushButton, QLineEdit,
    QTableWidget, QTableWidgetItem, QHeaderView, QTextEdit,
    QComboBox, QSpinBox, QDoubleSpinBox, QProgressBar, QGroupBox,
    QSplitter, QCheckBox, QDialog, QMessageBox, QFrame, QTabBar
)

# ROS 2 Imports
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
# 1. ROS 2 BACKEND WORKER NODE
# ==============================================================================
class RosBackendBridge(Node, QObject):
    # Signals to communicate thread-safely with PyQt5 UI
    new_log_signal = pyqtSignal(dict)
    new_incident_signal = pyqtSignal(dict)
    odom_update_signal = pyqtSignal(dict)
    motion_test_progress_signal = pyqtSignal(float, float, str) # current, target, status

    def __init__(self):
        Node.__init__(self, 'robot_diagnostics_gui_bridge')
        QObject.__init__(self)

        self.ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        self.log_dir = os.path.join(self.ws_root, 'log', 'diagnostics_sessions')
        os.makedirs(self.log_dir, exist_ok=True)

        self.session_start_time = datetime.now()
        self.session_id = self.session_start_time.strftime("%Y%m%d_%H%M%S")

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Sliding timestamp buffers
        self.scan_timestamps = deque(maxlen=40)
        self.odom_timestamps = deque(maxlen=60)
        self.cmd_vel_timestamps = deque(maxlen=40)
        self.raw_vel_timestamps = deque(maxlen=60)

        # Thread-safe State Cache
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

        # Missed loop & incidents
        self.incidents = []
        self.total_missed_loops = 0
        self.recent_logs = deque(maxlen=100)
        self.metrics_history = []

        # Motion Test State
        self.motion_test_active = False
        self.motion_test_type = 'forward' # 'forward', 'strafe', 'rotate'
        self.motion_start_x = 0.0
        self.motion_start_y = 0.0
        self.motion_start_theta = 0.0
        self.motion_target_dist = 1.0
        self.motion_test_speed = 0.15
        self.last_test_distance_recorded = 0.0

        # Dedicated Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reset_odom_pub = self.create_publisher(Empty, 'reset_odom', 10)

        # Dynamic Topics Subscriptions Dict: {topic_name: {'sub': sub, 'deque': deque, 'stats': dict}}
        self.dynamic_subs = {}

        # Setup standard subscriptions
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_sensor)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_reliable)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, qos_reliable)
        self.create_subscription(TwistStamped, '/raw_odom_vel', self._raw_vel_cb, qos_reliable)
        self.create_subscription(Log, '/rosout', self._rosout_cb, qos_reliable)

        # Periodic Timers
        self.create_timer(0.1, self._tf_benchmark_cb)       # 10 Hz TF Benchmark
        self.create_timer(0.05, self._motion_test_step_cb)  # 20 Hz Motion Test Driver
        self.create_timer(1.0, self._metrics_snapshot_cb)   # 1 Hz Snapshot for logging

    # ------------------ STANDARD TOPIC CALLBACKS ------------------
    def _scan_cb(self, msg: LaserScan):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delay_ms = (now_sec - stamp_sec) * 1000.0

        self.scan_timestamps.append(now_sec)
        rate_hz = 0.0
        if len(self.scan_timestamps) >= 2:
            dt = self.scan_timestamps[-1] - self.scan_timestamps[0]
            if dt > 0:
                rate_hz = (len(self.scan_timestamps) - 1) / dt

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
            if dt > 0:
                rate_hz = (len(self.odom_timestamps) - 1) / dt

        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        odom_dict = {
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

        with self.lock:
            self.latest_odom_stats = odom_dict

        self.odom_update_signal.emit(odom_dict)

    def _raw_vel_cb(self, msg: TwistStamped):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delay_ms = (now_sec - stamp_sec) * 1000.0

        self.raw_vel_timestamps.append(now_sec)
        rate_hz = 0.0
        if len(self.raw_vel_timestamps) >= 2:
            dt = self.raw_vel_timestamps[-1] - self.raw_vel_timestamps[0]
            if dt > 0:
                rate_hz = (len(self.raw_vel_timestamps) - 1) / dt

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
            if dt > 0:
                rate_hz = (len(self.cmd_vel_timestamps) - 1) / dt

        with self.lock:
            self.latest_cmd_vel_stats = {
                'rate_hz': round(rate_hz, 2),
                'vx': msg.linear.x,
                'vy': msg.linear.y,
                'wz': msg.angular.z,
                'last_rx': now_sec
            }

    def _rosout_cb(self, msg: Log):
        # Level: 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL
        now_iso = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        level_str = "INFO"
        if msg.level == 30: level_str = "WARN"
        elif msg.level >= 40: level_str = "ERROR"

        entry = {
            'time': now_iso,
            'node': msg.name,
            'level': level_str,
            'level_num': msg.level,
            'msg': msg.msg
        }

        with self.lock:
            self.recent_logs.append(entry)

        self.new_log_signal.emit(entry)

        # Detect controller_server missed loop
        if msg.level >= 30 and ("missed its desired rate" in msg.msg.lower() or "control loop missed" in msg.msg.lower()):
            self._record_incident(msg, now_iso)

    def _record_incident(self, log_msg: Log, timestamp_str: str):
        with self.lock:
            self.total_missed_loops += 1
            incident_id = self.total_missed_loops

            # Root cause analysis
            causes = []
            tf_mb = self.latest_tf_stats.get('map_to_base', {})
            tf_lookup_ms = tf_mb.get('lookup_duration_ms', 0.0)
            tf_age_ms = tf_mb.get('age_ms', 0.0)

            if tf_lookup_ms is not None and tf_lookup_ms > 35.0:
                causes.append(f"TF lookup 'map->base_link' BLOCKING {tf_lookup_ms:.1f}ms (>35% jatah loop 100ms)!")
            if tf_age_ms is not None and tf_age_ms > 120.0:
                causes.append(f"Transform 'map->base_link' kadaluarsa/basi ({tf_age_ms:.1f}ms)!")
            
            scan_delay = self.latest_scan_stats.get('delay_ms', 0.0)
            if scan_delay > 100.0:
                causes.append(f"Latensi scan LiDAR tinggi ({scan_delay:.1f}ms)")
            
            odom_rate = self.latest_odom_stats.get('rate_hz', 0.0)
            if odom_rate < 15.0 and odom_rate > 0.0:
                causes.append(f"Frekuensi /odom drop ke {odom_rate:.1f}Hz")

            cpu_pct = psutil.cpu_percent() if HAS_PSUTIL else None
            if cpu_pct is not None and cpu_pct > 80.0:
                causes.append(f"Beban CPU tinggi ({cpu_pct:.1f}%)")

            if not causes:
                causes.append("Kemungkinan contention lock pada Costmap update atau path planning compute.")

            incident = {
                'id': incident_id,
                'time': timestamp_str,
                'node': log_msg.name,
                'msg': log_msg.msg,
                'causes': causes,
                'snapshot': {
                    'cpu_pct': cpu_pct,
                    'tf': dict(self.latest_tf_stats),
                    'scan': dict(self.latest_scan_stats),
                    'odom': dict(self.latest_odom_stats)
                }
            }
            self.incidents.append(incident)

        self.new_incident_signal.emit(incident)

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
            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                dur_ms = (time.perf_counter() - t0) * 1000.0
                with self.lock:
                    self.latest_tf_stats[key] = {
                        'age_ms': None,
                        'lookup_duration_ms': round(dur_ms, 2),
                        'status': type(ex).__name__
                    }
            except Exception as e:
                dur_ms = (time.perf_counter() - t0) * 1000.0
                with self.lock:
                    self.latest_tf_stats[key] = {
                        'age_ms': None,
                        'lookup_duration_ms': round(dur_ms, 2),
                        'status': f"Err: {e}"
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

    # ------------------ MOTION TEST & JOG EXECUTION ------------------
    def start_motion_test(self, test_type: str, target_val: float, speed: float):
        with self.lock:
            self.motion_test_type = test_type
            self.motion_target_dist = abs(target_val)
            self.motion_test_speed = abs(speed)
            self.motion_start_x = self.latest_odom_stats['x']
            self.motion_start_y = self.latest_odom_stats['y']
            self.motion_start_theta = self.latest_odom_stats['theta']
            self.motion_test_active = True

        self.motion_test_progress_signal.emit(0.0, self.motion_target_dist, f"Memulai Uji {test_type.upper()}...")

    def stop_motion_test(self):
        with self.lock:
            self.motion_test_active = False
        self.send_cmd_vel(0.0, 0.0, 0.0)
        self.motion_test_progress_signal.emit(0.0, self.motion_target_dist, "Uji Dihentikan.")

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
        # Normalize angular diff
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

        if test_type == 'forward':
            traveled = math.sqrt(dx * dx + dy * dy)
            remain = target - traveled
            if remain <= 0.01:
                self.stop_motion_test()
                self.last_test_distance_recorded = traveled
                self.motion_test_progress_signal.emit(traveled, target, f"✅ Selesai! Jarak Odometri: {traveled:.3f} m")
                return
            self.send_cmd_vel(spd, 0.0, 0.0)
            self.motion_test_progress_signal.emit(traveled, target, f"Maju: {traveled:.3f} / {target:.3f} m")

        elif test_type == 'strafe':
            traveled = math.sqrt(dx * dx + dy * dy)
            remain = target - traveled
            if remain <= 0.01:
                self.stop_motion_test()
                self.last_test_distance_recorded = traveled
                self.motion_test_progress_signal.emit(traveled, target, f"✅ Selesai! Geser Odometri: {traveled:.3f} m")
                return
            self.send_cmd_vel(0.0, spd, 0.0)
            self.motion_test_progress_signal.emit(traveled, target, f"Geser Y: {traveled:.3f} / {target:.3f} m")

        elif test_type == 'rotate':
            traveled_deg = math.degrees(dtheta)
            remain = target - traveled_deg
            if remain <= 1.0:
                self.stop_motion_test()
                self.last_test_distance_recorded = traveled_deg
                self.motion_test_progress_signal.emit(traveled_deg, target, f"✅ Selesai! Rotasi Odometri: {traveled_deg:.1f}°")
                return
            self.send_cmd_vel(0.0, 0.0, spd)
            self.motion_test_progress_signal.emit(traveled_deg, target, f"Rotasi: {traveled_deg:.1f}° / {target:.1f}°")

    def send_cmd_vel(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def trigger_reset_odom(self):
        self.reset_odom_pub.publish(Empty())

    # ------------------ DYNAMIC TOPIC MONITORING ------------------
    def subscribe_dynamic_topic(self, topic_name: str, type_str: str, on_msg_cb):
        if topic_name in self.dynamic_subs:
            return True

        try:
            msg_cls = get_message(type_str)
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
            
            sub_info = {
                'timestamps': deque(maxlen=40),
                'total_count': 0,
                'last_delay_ms': None,
                'current_rate_hz': 0.0,
                'last_msg_yaml': ''
            }

            def internal_cb(msg):
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                sub_info['total_count'] += 1
                sub_info['timestamps'].append(now_sec)
                
                if len(sub_info['timestamps']) >= 2:
                    dt = sub_info['timestamps'][-1] - sub_info['timestamps'][0]
                    if dt > 0:
                        sub_info['current_rate_hz'] = (len(sub_info['timestamps']) - 1) / dt

                # Compute delay if header exists
                if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                    s_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    sub_info['last_delay_ms'] = (now_sec - s_sec) * 1000.0

                try:
                    yaml_text = message_to_yaml(msg)
                except Exception:
                    yaml_text = str(msg)
                sub_info['last_msg_yaml'] = yaml_text

                on_msg_cb(topic_name, sub_info, yaml_text)

            sub = self.create_subscription(msg_cls, topic_name, internal_cb, qos)
            self.dynamic_subs[topic_name] = {'sub': sub, 'info': sub_info}
            return True
        except Exception as e:
            self.get_logger().error(f"Gagal subscribe topik '{topic_name}': {e}")
            return False

    def unsubscribe_dynamic_topic(self, topic_name: str):
        if topic_name in self.dynamic_subs:
            sub_obj = self.dynamic_subs[topic_name]['sub']
            self.destroy_subscription(sub_obj)
            del self.dynamic_subs[topic_name]

    # ------------------ RUNTIME PARAMETER CLIENT ------------------
    def set_odometry_scale_param(self, param_name: str, new_val: float):
        """Mengirim request update parameter ke /odometry_node via service ROS 2."""
        cli = self.create_client(SetParameters, '/odometry_node/set_parameters')
        if not cli.wait_for_service(timeout_sec=1.5):
            return False, "Service '/odometry_node/set_parameters' tidak tersedia."

        req = SetParameters.Request()
        p = Parameter()
        p.name = param_name
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = float(new_val)
        req.parameters = [p]

        future = cli.call_async(req)
        # Tunggu maksimal 1.5 detik
        t0 = time.time()
        while not future.done() and (time.time() - t0 < 1.5):
            time.sleep(0.05)

        if future.done():
            res = future.result()
            if res and res.results and res.results[0].successful:
                return True, f"Parameter '{param_name}' berhasil diubah ke {new_val:.4f}"
            else:
                return False, f"Gagal mengubah parameter: {res.results[0].reason if res.results else 'Unknown'}"
        return False, "Timeout saat memanggil parameter service."


# ==============================================================================
# 2. DIALOG: TAMBAH TAB TOPIK
# ==============================================================================
class AddTopicDialog(QDialog):
    def __init__(self, ros_node: RosBackendBridge, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node
        self.selected_topic = None
        self.selected_type = None

        self.setWindowTitle("➕ Tambah Tab Monitoring Topik")
        self.resize(550, 400)
        self.setStyleSheet("""
            QDialog { background-color: #1e1e2e; color: #cdd6f4; }
            QLabel { color: #cdd6f4; font-size: 13px; font-weight: bold; }
            QLineEdit, QComboBox { background-color: #2b2b3d; color: #ffffff; border: 1px solid #45475a; border-radius: 4px; padding: 6px; }
            QTableWidget { background-color: #2b2b3d; color: #ffffff; gridline-color: #45475a; border: 1px solid #45475a; }
            QHeaderView::section { background-color: #313244; color: #89b4fa; padding: 4px; font-weight: bold; border: 1px solid #45475a; }
            QPushButton { background-color: #89b4fa; color: #11111b; font-weight: bold; border-radius: 4px; padding: 7px 14px; }
            QPushButton:hover { background-color: #b4befe; }
        """)

        layout = QVBoxLayout(self)

        layout.addWidget(QLabel("Pilih Topik Aktif dari ROS 2 Network:"))
        
        # Search / filter
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText("🔍 Ketik nama topik untuk memfilter...")
        self.filter_edit.textChanged.connect(self._filter_topics)
        layout.addWidget(self.filter_edit)

        # Table of topics
        self.topic_table = QTableWidget()
        self.topic_table.setColumnCount(2)
        self.topic_table.setHorizontalHeaderLabels(["Nama Topik", "Tipe Pesan"])
        self.topic_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.topic_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.topic_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.topic_table.setSelectionMode(QTableWidget.SingleSelection)
        self.topic_table.itemSelectionChanged.connect(self._on_row_selected)
        layout.addWidget(self.topic_table)

        btn_layout = QHBoxLayout()
        self.btn_refresh = QPushButton("🔄 Refresh Topik")
        self.btn_refresh.clicked.connect(self._populate_topics)
        self.btn_open = QPushButton("✅ Buka Tab Topik Ini")
        self.btn_open.setEnabled(False)
        self.btn_open.clicked.connect(self.accept)
        self.btn_cancel = QPushButton("Batal")
        self.btn_cancel.clicked.connect(self.reject)

        btn_layout.addWidget(self.btn_refresh)
        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_cancel)
        btn_layout.addWidget(self.btn_open)
        layout.addLayout(btn_layout)

        self._populate_topics()

    def _populate_topics(self):
        self.topic_table.setRowCount(0)
        try:
            topics = self.ros_node.get_topic_names_and_types()
            for row_idx, (t_name, t_types) in enumerate(sorted(topics, key=lambda x: x[0])):
                self.topic_table.insertRow(row_idx)
                item_name = QTableWidgetItem(t_name)
                item_type = QTableWidgetItem(t_types[0] if t_types else 'Unknown')
                self.topic_table.setItem(row_idx, 0, item_name)
                self.topic_table.setItem(row_idx, 1, item_type)
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Gagal mengambil topik ROS 2: {e}")

    def _filter_topics(self, text):
        for r in range(self.topic_table.rowCount()):
            name = self.topic_table.item(r, 0).text().lower()
            typ = self.topic_table.item(r, 1).text().lower()
            match = (text.lower() in name) or (text.lower() in typ)
            self.topic_table.setRowHidden(r, not match)

    def _on_row_selected(self):
        selected_rows = self.topic_table.selectedItems()
        if selected_rows:
            row = selected_rows[0].row()
            self.selected_topic = self.topic_table.item(row, 0).text()
            self.selected_type = self.topic_table.item(row, 1).text()
            self.btn_open.setEnabled(True)
        else:
            self.btn_open.setEnabled(False)


# ==============================================================================
# 3. WIDGET: DYNAMIC TOPIC MONITOR TAB
# ==============================================================================
class DynamicTopicTab(QWidget):
    def __init__(self, topic_name: str, type_str: str, ros_node: RosBackendBridge, parent=None):
        super().__init__(parent)
        self.topic_name = topic_name
        self.type_str = type_str
        self.ros_node = ros_node
        self.is_paused = False
        self.rates_history = deque(maxlen=30)

        layout = QVBoxLayout(self)

        # Header Stats
        header_frame = QFrame()
        header_frame.setStyleSheet("background-color: #2b2b3d; border-radius: 6px; padding: 6px;")
        h_layout = QHBoxLayout(header_frame)

        self.lbl_topic = QLabel(f"<b>Topik:</b> <font color='#89b4fa'>{topic_name}</font>  |  <b>Tipe:</b> <font color='#a6adc8'>{type_str}</font>")
        self.lbl_rate = QLabel("<b>Freq:</b> <font color='#a6e3a1'>-- Hz</font>")
        self.lbl_delay = QLabel("<b>Delay:</b> <font color='#fab387'>-- ms</font>")
        self.lbl_count = QLabel("<b>Msgs:</b> 0")

        self.btn_pause = QPushButton("⏸️ Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.clicked.connect(self._toggle_pause)

        self.btn_clear = QPushButton("🧹 Clear")
        self.btn_clear.clicked.connect(self._clear_text)

        h_layout.addWidget(self.lbl_topic)
        h_layout.addStretch()
        h_layout.addWidget(self.lbl_rate)
        h_layout.addWidget(self.lbl_delay)
        h_layout.addWidget(self.lbl_count)
        h_layout.addWidget(self.btn_pause)
        h_layout.addWidget(self.btn_clear)
        layout.addWidget(header_frame)

        # Content Inspector (YAML Formatted)
        self.text_inspector = QTextEdit()
        self.text_inspector.setReadOnly(True)
        self.text_inspector.setFont(QFont("Monospace", 10))
        self.text_inspector.setStyleSheet("""
            QTextEdit { background-color: #181825; color: #cdd6f4; border: 1px solid #45475a; border-radius: 4px; padding: 6px; }
        """)
        layout.addWidget(self.text_inspector)

        # Auto-scroll checkbox
        self.cb_autoscroll = QCheckBox("Auto-scroll ke data terbaru")
        self.cb_autoscroll.setChecked(True)
        layout.addWidget(self.cb_autoscroll)

        # Subscribe via backend
        self.ros_node.subscribe_dynamic_topic(self.topic_name, self.type_str, self.on_new_message)

    def on_new_message(self, topic_name, sub_info, yaml_text):
        if self.is_paused:
            return
        # Schedule update on UI thread
        rate = sub_info.get('current_rate_hz', 0.0)
        delay = sub_info.get('last_delay_ms')
        count = sub_info.get('total_count', 0)

        self.lbl_rate.setText(f"<b>Freq:</b> <font color='#a6e3a1'>{rate:.1f} Hz</font>")
        if delay is not None:
            self.lbl_delay.setText(f"<b>Delay:</b> <font color='#fab387'>{delay:.1f} ms</font>")
        else:
            self.lbl_delay.setText("<b>Delay:</b> <font color='#6c7086'>N/A</font>")
        self.lbl_count.setText(f"<b>Msgs:</b> {count}")

        now_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        display_block = f"--- [Timestamp: {now_str}] ---\n{yaml_text}\n"
        self.text_inspector.setPlainText(display_block)

        if self.cb_autoscroll.isChecked():
            sb = self.text_inspector.verticalScrollBar()
            sb.setValue(sb.maximum())

    def _toggle_pause(self):
        self.is_paused = self.btn_pause.isChecked()
        self.btn_pause.setText("▶️ Resume" if self.is_paused else "⏸️ Pause")

    def _clear_text(self):
        self.text_inspector.clear()

    def close(self):
        self.ros_node.unsubscribe_dynamic_topic(self.topic_name)
        super().close()


# ==============================================================================
# 4. MAIN WINDOW: NONAME ROBOT MONITOR & CALIBRATION SUITE
# ==============================================================================
class DiagnosticsMainWindow(QMainWindow):
    def __init__(self, ros_node: RosBackendBridge):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("🤖 NonameRobot - Diagnostics, Telemetry & Odometry Calibration Suite")
        self.resize(1280, 850)
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e2e; }
            QTabWidget::pane { border: 1px solid #45475a; background-color: #1e1e2e; border-radius: 4px; }
            QTabBar::tab { background-color: #2b2b3d; color: #a6adc8; padding: 9px 18px; font-weight: bold; border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 3px; }
            QTabBar::tab:selected { background-color: #3b3b54; color: #89b4fa; border-bottom: 2px solid #89b4fa; }
            QTabBar::tab:hover { background-color: #313244; color: #cdd6f4; }
            QGroupBox { font-size: 13px; font-weight: bold; color: #89b4fa; border: 1px solid #45475a; border-radius: 6px; margin-top: 10px; padding-top: 15px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QLabel { color: #cdd6f4; font-size: 12px; }
            QPushButton { background-color: #3b3b54; color: #ffffff; font-weight: bold; border-radius: 5px; padding: 6px 12px; border: 1px solid #585b70; }
            QPushButton:hover { background-color: #45475a; }
            QPushButton:pressed { background-color: #585b70; }
            QLineEdit, QDoubleSpinBox, QComboBox { background-color: #252538; color: #ffffff; border: 1px solid #45475a; border-radius: 4px; padding: 5px; }
            QTableWidget { background-color: #181825; color: #cdd6f4; gridline-color: #313244; border: 1px solid #45475a; border-radius: 4px; }
            QHeaderView::section { background-color: #252538; color: #89b4fa; padding: 5px; font-weight: bold; border: 1px solid #313244; }
        """)

        # Central Widget & Tabs
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # Top Bar
        top_bar = self._create_top_bar()
        main_layout.addLayout(top_bar)

        # Tab Widget
        self.tabs = QTabWidget()
        self.tabs.setTabsClosable(True)
        self.tabs.tabCloseRequested.connect(self._close_tab)
        main_layout.addWidget(self.tabs)

        # Tab 1: Dashboard Utama
        self.tab_dashboard = self._create_dashboard_tab()
        self.tabs.addTab(self.tab_dashboard, "📊 Dashboard Utama")
        self.tabs.tabBar().setTabButton(0, QTabBar.RightSide, None) # Tab 1 cannot be closed

        # Tab 2: Kalibrasi & Uji Odometri
        self.tab_calibration = self._create_calibration_tab()
        self.tabs.addTab(self.tab_calibration, "🎯 Kalibrasi & Uji Odometri")
        self.tabs.tabBar().setTabButton(1, QTabBar.RightSide, None) # Tab 2 cannot be closed

        # Connect Signals from Backend
        self.ros_node.new_log_signal.connect(self._on_new_log)
        self.ros_node.new_incident_signal.connect(self._on_new_incident)
        self.ros_node.odom_update_signal.connect(self._on_odom_update)
        self.ros_node.motion_test_progress_signal.connect(self._on_motion_test_progress)

        # Main UI Refresh Timer (10 Hz)
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self._refresh_ui)
        self.ui_timer.start(100)

    # ==================== TOP BAR ====================
    def _create_top_bar(self):
        layout = QHBoxLayout()

        self.lbl_title = QLabel("🤖 <b>NONAME ROBOT DIAGNOSTICS</b>")
        self.lbl_title.setStyleSheet("font-size: 16px; color: #89b4fa;")

        self.lbl_session = QLabel(f"Sesi: {self.ros_node.session_id}")
        self.lbl_session.setStyleSheet("color: #a6adc8; font-size: 12px;")

        self.lbl_sys_stats = QLabel("CPU: --% | RAM: --/-- GB")
        self.lbl_sys_stats.setStyleSheet("background-color: #252538; padding: 4px 10px; border-radius: 4px; color: #a6e3a1;")

        btn_add_tab = QPushButton("➕ Tambah Tab Topic")
        btn_add_tab.setStyleSheet("background-color: #89b4fa; color: #11111b;")
        btn_add_tab.clicked.connect(self._open_add_topic_dialog)

        btn_save_session = QPushButton("💾 Simpan Sesi")
        btn_save_session.setStyleSheet("background-color: #a6e3a1; color: #11111b;")
        btn_save_session.clicked.connect(self._save_session)

        layout.addWidget(self.lbl_title)
        layout.addWidget(self.lbl_session)
        layout.addStretch()
        layout.addWidget(self.lbl_sys_stats)
        layout.addWidget(btn_add_tab)
        layout.addWidget(btn_save_session)

        return layout

    # ==================== TAB 1: DASHBOARD UTAMA ====================
    def _create_dashboard_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Split horizontal: Left (Topics + TF + Incidents) / Right (Live Logs)
        splitter = QSplitter(Qt.Horizontal)

        # --- LEFT PANEL ---
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)

        # 1. Health Topik Kritis
        grp_topics = QGroupBox("1. KESEGARAN & FREKUENSI TOPIK UTAMA")
        g_layout = QVBoxLayout(grp_topics)

        self.table_topics = QTableWidget()
        self.table_topics.setColumnCount(5)
        self.table_topics.setHorizontalHeaderLabels(["Topik", "Frekuensi", "Delay (ms)", "Status", "Payload / Info"])
        self.table_topics.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_topics.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table_topics.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table_topics.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table_topics.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        self.table_topics.setRowCount(4)

        topics_init = [
            ("/scan (LiDAR)", "-- Hz", "-- ms", "WAIT", "--"),
            ("/odom (Roda)", "-- Hz", "-- ms", "WAIT", "--"),
            ("/cmd_vel (Controller)", "-- Hz", "-- ms", "WAIT", "--"),
            ("/raw_odom_vel (Low-level)", "-- Hz", "-- ms", "WAIT", "--")
        ]
        for row, (name, f, d, s, p) in enumerate(topics_init):
            self.table_topics.setItem(row, 0, QTableWidgetItem(name))
            self.table_topics.setItem(row, 1, QTableWidgetItem(f))
            self.table_topics.setItem(row, 2, QTableWidgetItem(d))
            self.table_topics.setItem(row, 3, QTableWidgetItem(s))
            self.table_topics.setItem(row, 4, QTableWidgetItem(p))
        g_layout.addWidget(self.table_topics)
        left_layout.addWidget(grp_topics)

        # 2. TF Latency & Benchmark
        grp_tf = QGroupBox("2. LATENSI & DURASI LOOKUP TF (MAP -> ODOM -> BASE_LINK)")
        tf_layout = QGridLayout(grp_tf)

        self.lbl_tf_odom_base = QLabel("<b>odom ➜ base_link:</b> Age: -- ms | Dur: -- ms [N/A]")
        self.lbl_tf_map_odom = QLabel("<b>map ➜ odom:</b> Age: -- ms | Dur: -- ms [N/A]")
        self.lbl_tf_map_base = QLabel("<b>map ➜ base_link:</b> Age: -- ms | Dur: -- ms [N/A]")

        tf_layout.addWidget(self.lbl_tf_odom_base, 0, 0)
        tf_layout.addWidget(self.lbl_tf_map_odom, 1, 0)
        tf_layout.addWidget(self.lbl_tf_map_base, 2, 0)
        left_layout.addWidget(grp_tf)

        # 3. Missed-Loop Incident Recorder
        grp_incidents = QGroupBox("3. MISSED-LOOP INCIDENTS RECORDER (10Hz CONTROLLER SERVER)")
        inc_layout = QVBoxLayout(grp_incidents)

        h_inc = QHBoxLayout()
        self.lbl_missed_badge = QLabel("<b>Total Missed Loops: 0</b>")
        self.lbl_missed_badge.setStyleSheet("background-color: #a6e3a1; color: #11111b; padding: 4px 10px; border-radius: 4px; font-weight: bold;")
        h_inc.addWidget(self.lbl_missed_badge)
        h_inc.addStretch()
        inc_layout.addLayout(h_inc)

        self.table_incidents = QTableWidget()
        self.table_incidents.setColumnCount(4)
        self.table_incidents.setHorizontalHeaderLabels(["ID", "Waktu", "Pesan", "Penyebab Teridentifikasi"])
        self.table_incidents.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_incidents.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table_incidents.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table_incidents.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        inc_layout.addWidget(self.table_incidents)

        left_layout.addWidget(grp_incidents)
        splitter.addWidget(left_widget)

        # --- RIGHT PANEL (ROSOUT CONSOLE) ---
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)

        grp_logs = QGroupBox("4. LIVE ROS 2 WARNING & ERROR CONSOLE (/rosout)")
        log_layout = QVBoxLayout(grp_logs)

        h_filter = QHBoxLayout()
        self.cb_filter_level = QComboBox()
        self.cb_filter_level.addItems(["SEMUA (WARN + ERROR)", "HANYA WARN", "HANYA ERROR"])
        self.cb_filter_level.currentIndexChanged.connect(self._filter_logs)

        self.btn_clear_log = QPushButton("🧹 Clear Logs")
        self.btn_clear_log.clicked.connect(self._clear_logs_table)

        h_filter.addWidget(QLabel("Filter:"))
        h_filter.addWidget(self.cb_filter_level)
        h_filter.addStretch()
        h_filter.addWidget(self.btn_clear_log)
        log_layout.addLayout(h_filter)

        self.table_logs = QTableWidget()
        self.table_logs.setColumnCount(4)
        self.table_logs.setHorizontalHeaderLabels(["Waktu", "Level", "Node", "Pesan"])
        self.table_logs.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_logs.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table_logs.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table_logs.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        log_layout.addWidget(self.table_logs)

        right_layout.addWidget(grp_logs)
        splitter.addWidget(right_widget)

        # Proportions: 55% left, 45% right
        splitter.setSizes([700, 500])
        layout.addWidget(splitter)

        return widget

    # ==================== TAB 2: KALIBRASI & UJI ODOMETRI ====================
    def _create_calibration_tab(self):
        widget = QWidget()
        layout = QHBoxLayout(widget)

        # Left Panel: Live Odom Display & Manual Jog & Motion Test
        left_group = QGroupBox("1. KONTROL GERAK & MONITOR ODOMETRI")
        left_layout = QVBoxLayout(left_group)

        # Live Display Cards
        disp_card = QFrame()
        disp_card.setStyleSheet("background-color: #181825; border-radius: 6px; padding: 10px;")
        disp_grid = QGridLayout(disp_card)

        self.lbl_odom_x = QLabel("<b>X (m):</b> <font color='#89b4fa' size='+1'>0.000</font>")
        self.lbl_odom_y = QLabel("<b>Y (m):</b> <font color='#89b4fa' size='+1'>0.000</font>")
        self.lbl_odom_theta = QLabel("<b>Yaw:</b> <font color='#89b4fa' size='+1'>0.0° (0.00 rad)</font>")
        self.lbl_odom_vel = QLabel("<b>Kecepatan:</b> Vx: 0.00 | Vy: 0.00 | Wz: 0.00")
        self.lbl_rel_distance = QLabel("<b>Jarak Tempuh Sejak Zero:</b> <font color='#a6e3a1' size='+1'>0.000 m</font>")

        disp_grid.addWidget(self.lbl_odom_x, 0, 0)
        disp_grid.addWidget(self.lbl_odom_y, 0, 1)
        disp_grid.addWidget(self.lbl_odom_theta, 0, 2)
        disp_grid.addWidget(self.lbl_odom_vel, 1, 0, 1, 2)
        disp_grid.addWidget(self.lbl_rel_distance, 1, 2)
        left_layout.addWidget(disp_card)

        # Zero / Tare Buttons
        zero_layout = QHBoxLayout()
        self.btn_tare_gui = QPushButton("📍 Set Software Zero (GUI)")
        self.btn_tare_gui.clicked.connect(self._tare_software_zero)

        self.btn_reset_node_odom = QPushButton("🔄 Reset Odometri Node (/reset_odom)")
        self.btn_reset_node_odom.setStyleSheet("background-color: #fab387; color: #11111b; font-weight: bold;")
        self.btn_reset_node_odom.clicked.connect(self._trigger_node_odom_reset)

        zero_layout.addWidget(self.btn_tare_gui)
        zero_layout.addWidget(self.btn_reset_node_odom)
        left_layout.addLayout(zero_layout)

        # Motion Test Box
        test_box = QGroupBox("Uji Gerak Otomatis (Motion Distance Test)")
        t_layout = QGridLayout(test_box)

        t_layout.addWidget(QLabel("Tipe Uji:"), 0, 0)
        self.combo_test_type = QComboBox()
        self.combo_test_type.addItem("➡️ Uji Maju Sumbu X (Linear X)", "forward")
        self.combo_test_type.addItem("⬆️ Uji Geser Sumbu Y (Strafe Y)", "strafe")
        self.combo_test_type.addItem("🔄 Uji Rotasi Sudut (Yaw Z)", "rotate")
        t_layout.addWidget(self.combo_test_type, 0, 1)

        t_layout.addWidget(QLabel("Target Nilai:"), 1, 0)
        self.spin_test_target = QDoubleSpinBox()
        self.spin_test_target.setRange(0.1, 10.0)
        self.spin_test_target.setValue(1.000)
        self.spin_test_target.setSingleStep(0.1)
        self.spin_test_target.setSuffix(" m")
        t_layout.addWidget(self.spin_test_target, 1, 1)

        t_layout.addWidget(QLabel("Kecepatan Uji:"), 2, 0)
        self.spin_test_speed = QDoubleSpinBox()
        self.spin_test_speed.setRange(0.05, 0.5)
        self.spin_test_speed.setValue(0.15)
        self.spin_test_speed.setSingleStep(0.05)
        self.spin_test_speed.setSuffix(" m/s")
        t_layout.addWidget(self.spin_test_speed, 2, 1)

        self.progress_motion = QProgressBar()
        self.progress_motion.setValue(0)
        t_layout.addWidget(self.progress_motion, 3, 0, 1, 2)

        self.lbl_motion_status = QLabel("Status: Siap.")
        t_layout.addWidget(self.lbl_motion_status, 4, 0, 1, 2)

        h_test_btns = QHBoxLayout()
        self.btn_start_test = QPushButton("▶️ Jalankan Uji Gerak")
        self.btn_start_test.setStyleSheet("background-color: #a6e3a1; color: #11111b; font-weight: bold; padding: 8px;")
        self.btn_start_test.clicked.connect(self._run_motion_test)

        self.btn_stop_test = QPushButton("🛑 EMERGENCY STOP (E-STOP)")
        self.btn_stop_test.setStyleSheet("background-color: #f38ba8; color: #11111b; font-weight: bold; padding: 8px;")
        self.btn_stop_test.clicked.connect(self._estop)

        h_test_btns.addWidget(self.btn_start_test)
        h_test_btns.addWidget(self.btn_stop_test)
        t_layout.addLayout(h_test_btns, 5, 0, 1, 2)

        left_layout.addWidget(test_box)

        # Manual Jog D-Pad
        jog_box = QGroupBox("Manual Jog Controller (Posisioning)")
        j_layout = QGridLayout(jog_box)

        btn_fwd = QPushButton("▲ Maju")
        btn_back = QPushButton("▼ Mundur")
        btn_left = QPushButton("◀ Geser Kiri")
        btn_right = QPushButton("▶ Geser Kanan")
        btn_rot_ccw = QPushButton("↺ Putar CCW")
        btn_rot_cw = QPushButton("↻ Putar CW")
        btn_stop = QPushButton("⏹ STOP")
        btn_stop.setStyleSheet("background-color: #f38ba8; color: #11111b;")

        btn_fwd.clicked.connect(lambda: self.ros_node.send_cmd_vel(0.2, 0.0, 0.0))
        btn_back.clicked.connect(lambda: self.ros_node.send_cmd_vel(-0.2, 0.0, 0.0))
        btn_left.clicked.connect(lambda: self.ros_node.send_cmd_vel(0.0, 0.2, 0.0))
        btn_right.clicked.connect(lambda: self.ros_node.send_cmd_vel(0.0, -0.2, 0.0))
        btn_rot_ccw.clicked.connect(lambda: self.ros_node.send_cmd_vel(0.0, 0.0, 0.5))
        btn_rot_cw.clicked.connect(lambda: self.ros_node.send_cmd_vel(0.0, 0.0, -0.5))
        btn_stop.clicked.connect(lambda: self.ros_node.send_cmd_vel(0.0, 0.0, 0.0))

        j_layout.addWidget(btn_rot_ccw, 0, 0)
        j_layout.addWidget(btn_fwd, 0, 1)
        j_layout.addWidget(btn_rot_cw, 0, 2)
        j_layout.addWidget(btn_left, 1, 0)
        j_layout.addWidget(btn_stop, 1, 1)
        j_layout.addWidget(btn_right, 1, 2)
        j_layout.addWidget(btn_back, 2, 1)

        left_layout.addWidget(jog_box)
        layout.addWidget(left_group, 5)

        # Right Panel: Scale Calibration Calculator
        right_group = QGroupBox("2. KALKULATOR KALIBRASI SKALA ODOMETRI")
        right_layout = QVBoxLayout(right_group)

        info_box = QLabel(
            "💡 <b>Panduan Mengatasi Pergeseran Scan Laser:</b><br>"
            "Jika saat robot bergerak maju lurus titik scan laser <b>bergeser mendekati robot</b> "
            "(lepas dari dinding hitam peta), berarti odometri mengalami <b>under-counting</b> "
            "(jarak terbaca < jarak fisik sebenarnya).<br><br>"
            "Lakukan uji gerak 1.0 meter fisik dengan meteran, lalu hitung dan terapkan faktor skala baru di bawah ini."
        )
        info_box.setWordWrap(True)
        info_box.setStyleSheet("background-color: #252538; padding: 10px; border-radius: 6px; color: #cdd6f4;")
        right_layout.addWidget(info_box)

        form_grid = QGridLayout()

        form_grid.addWidget(QLabel("<b>Pilih Sumbu Kalibrasi:</b>"), 0, 0)
        self.combo_axis = QComboBox()
        self.combo_axis.addItem("linear_scale_x (Maju / Mundur)", "linear_scale_x")
        self.combo_axis.addItem("linear_scale_y (Geser Kiri / Kanan)", "linear_scale_y")
        self.combo_axis.addItem("angular_scale_z (Rotasi Yaw)", "angular_scale_z")
        form_grid.addWidget(self.combo_axis, 0, 1)

        form_grid.addWidget(QLabel("<b>Skala Saat Ini:</b>"), 1, 0)
        self.spin_current_scale = QDoubleSpinBox()
        self.spin_current_scale.setRange(0.1, 5.0)
        self.spin_current_scale.setValue(1.05)
        self.spin_current_scale.setDecimals(4)
        self.spin_current_scale.setSingleStep(0.01)
        form_grid.addWidget(self.spin_current_scale, 1, 1)

        form_grid.addWidget(QLabel("<b>Jarak Terbaca Odometri:</b>"), 2, 0)
        h_odom_in = QHBoxLayout()
        self.spin_odom_dist = QDoubleSpinBox()
        self.spin_odom_dist.setRange(0.001, 20.0)
        self.spin_odom_dist.setValue(0.870)
        self.spin_odom_dist.setDecimals(4)
        self.spin_odom_dist.setSuffix(" m")
        self.btn_copy_last = QPushButton("📥 Ambil Hasil Uji")
        self.btn_copy_last.clicked.connect(self._copy_last_test_distance)
        h_odom_in.addWidget(self.spin_odom_dist)
        h_odom_in.addWidget(self.btn_copy_last)
        form_grid.addLayout(h_odom_in, 2, 1)

        form_grid.addWidget(QLabel("<b>Jarak Fisik Aktual (Meteran):</b>"), 3, 0)
        self.spin_actual_dist = QDoubleSpinBox()
        self.spin_actual_dist.setRange(0.001, 20.0)
        self.spin_actual_dist.setValue(1.000)
        self.spin_actual_dist.setDecimals(4)
        self.spin_actual_dist.setSuffix(" m")
        form_grid.addWidget(self.spin_actual_dist, 3, 1)

        right_layout.addLayout(form_grid)

        # Button Calculate
        self.btn_calc_scale = QPushButton("🧮 Hitung Skala Baru")
        self.btn_calc_scale.setStyleSheet("background-color: #89b4fa; color: #11111b; font-weight: bold; padding: 8px;")
        self.btn_calc_scale.clicked.connect(self._calculate_scale)
        right_layout.addWidget(self.btn_calc_scale)

        # Calculation Result Card
        res_card = QFrame()
        res_card.setStyleSheet("background-color: #181825; border-radius: 6px; padding: 12px; border: 1px solid #45475a;")
        rc_layout = QVBoxLayout(res_card)

        self.lbl_calc_formula = QLabel("Rumus: Skala Baru = Skala Lama × (Jarak Aktual / Jarak Odom)")
        self.lbl_calc_formula.setStyleSheet("color: #a6adc8; font-style: italic;")
        self.lbl_calc_result = QLabel("<b>Skala Baru Rekomendasi: <font color='#a6e3a1' size='+2'>--</font></b>")
        self.lbl_calc_delta = QLabel("Koreksi: --%")

        rc_layout.addWidget(self.lbl_calc_formula)
        rc_layout.addWidget(self.lbl_calc_result)
        rc_layout.addWidget(self.lbl_calc_delta)
        right_layout.addWidget(res_card)

        # Apply Buttons
        apply_layout = QVBoxLayout()
        self.btn_apply_runtime = QPushButton("🚀 Terapkan Langsung ke /odometry_node (Runtime)")
        self.btn_apply_runtime.setStyleSheet("background-color: #a6e3a1; color: #11111b; font-weight: bold; padding: 9px;")
        self.btn_apply_runtime.clicked.connect(self._apply_runtime_scale)

        self.btn_save_config = QPushButton("💾 Simpan Permanen ke nav2_params_paper.yaml")
        self.btn_save_config.setStyleSheet("background-color: #f9e2af; color: #11111b; font-weight: bold; padding: 9px;")
        self.btn_save_config.clicked.connect(self._save_scale_to_yaml)

        apply_layout.addWidget(self.btn_apply_runtime)
        apply_layout.addWidget(self.btn_save_config)
        right_layout.addLayout(apply_layout)

        right_layout.addStretch()
        layout.addWidget(right_group, 5)

        # Initial zero tracking
        self.tare_x = 0.0
        self.tare_y = 0.0
        self.calculated_new_scale = None

        return widget

    # ==================== TAB MANAGEMENT ====================
    def _open_add_topic_dialog(self):
        dlg = AddTopicDialog(self.ros_node, self)
        if dlg.exec_() == QDialog.Accepted and dlg.selected_topic:
            t_name = dlg.selected_topic
            t_type = dlg.selected_type
            # Check if tab already exists
            for idx in range(self.tabs.count()):
                if self.tabs.tabText(idx) == t_name:
                    self.tabs.setCurrentIndex(idx)
                    return

            new_tab = DynamicTopicTab(t_name, t_type, self.ros_node, self)
            new_idx = self.tabs.addTab(new_tab, t_name)
            self.tabs.setCurrentIndex(new_idx)

    def _close_tab(self, index):
        if index in (0, 1):
            return # Protect main tabs
        widget = self.tabs.widget(index)
        if isinstance(widget, DynamicTopicTab):
            widget.close()
        self.tabs.removeTab(index)

    # ==================== PERIODIC UI REFRESH ====================
    def _refresh_ui(self):
        # 1. Host Stats
        if HAS_PSUTIL:
            cpu_pct = psutil.cpu_percent()
            mem = psutil.virtual_memory()
            mem_str = f"{mem.used / 1e9:.1f}/{mem.total / 1e9:.1f} GB ({mem.percent}%)"
            self.lbl_sys_stats.setText(f"CPU: {cpu_pct:4.1f}% | RAM: {mem_str}")

        # 2. Topic Health Table
        with self.ros_node.lock:
            scan = dict(self.ros_node.latest_scan_stats)
            odom = dict(self.ros_node.latest_odom_stats)
            cmd = dict(self.ros_node.latest_cmd_vel_stats)
            raw = dict(self.ros_node.latest_raw_vel_stats)
            tfs = dict(self.ros_node.latest_tf_stats)

        # /scan
        self._set_topic_row(0, f"{scan['rate_hz']:.1f} Hz", f"{scan['delay_ms']:.1f} ms",
                            "OK" if scan['rate_hz'] > 5.0 else "SLOW", f"Points: {scan['valid']}/{scan['pts']}")
        # /odom
        self._set_topic_row(1, f"{odom['rate_hz']:.1f} Hz", f"{odom['delay_ms']:.1f} ms",
                            "OK" if odom['rate_hz'] > 15.0 else "SLOW", f"Vx: {odom['vx']:.2f}, Vy: {odom['vy']:.2f}, Wz: {odom['wz']:.2f}")
        # /cmd_vel
        self._set_topic_row(2, f"{cmd['rate_hz']:.1f} Hz", "--",
                            "ACTIVE" if cmd['rate_hz'] > 0.0 else "IDLE", f"Vx: {cmd['vx']:.2f}, Vy: {cmd['vy']:.2f}, Wz: {cmd['wz']:.2f}")
        # /raw_odom_vel
        self._set_topic_row(3, f"{raw['rate_hz']:.1f} Hz", f"{raw['delay_ms']:.1f} ms",
                            "OK" if raw['rate_hz'] > 15.0 else "WAIT", f"Vx: {raw['vx']:.2f}, Vy: {raw['vy']:.2f}, Wz: {raw['wz']:.2f}")

        # 3. TF Benchmark Cards
        self._update_tf_labels(tfs)

    def _set_topic_row(self, row, freq, delay, status, info):
        self.table_topics.item(row, 1).setText(freq)
        self.table_topics.item(row, 2).setText(delay)
        st_item = self.table_topics.item(row, 3)
        st_item.setText(status)
        if status in ("OK", "ACTIVE"):
            st_item.setForeground(QColor("#a6e3a1"))
        elif status in ("SLOW", "WAIT"):
            st_item.setForeground(QColor("#fab387"))
        else:
            st_item.setForeground(QColor("#f38ba8"))
        self.table_topics.item(row, 4).setText(info)

    def _update_tf_labels(self, tfs):
        ob = tfs.get('odom_to_base', {})
        mo = tfs.get('map_to_odom', {})
        mb = tfs.get('map_to_base', {})

        def fmt(d):
            age = f"{d.get('age_ms', 'N/A')} ms" if d.get('age_ms') is not None else "N/A"
            dur = f"{d.get('lookup_duration_ms', 0.0):.1f} ms"
            st = d.get('status', 'N/A')
            color = "#a6e3a1" if st == 'OK' else "#fab387"
            return f"Age: <font color='{color}'>{age}</font> | Dur: {dur} [{st}]"

        self.lbl_tf_odom_base.setText(f"<b>odom ➜ base_link:</b> {fmt(ob)}")
        self.lbl_tf_map_odom.setText(f"<b>map ➜ odom:</b> {fmt(mo)}")
        self.lbl_tf_map_base.setText(f"<b>map ➜ base_link:</b> {fmt(mb)}")

    # ==================== LOG & INCIDENT HANDLERS ====================
    def _on_new_log(self, entry: dict):
        row = self.table_logs.rowCount()
        self.table_logs.insertRow(row)
        self.table_logs.setItem(row, 0, QTableWidgetItem(entry['time']))
        
        lvl_item = QTableWidgetItem(entry['level'])
        if entry['level'] == 'ERROR':
            lvl_item.setForeground(QColor("#f38ba8"))
        elif entry['level'] == 'WARN':
            lvl_item.setForeground(QColor("#fab387"))
        self.table_logs.setItem(row, 1, lvl_item)

        self.table_logs.setItem(row, 2, QTableWidgetItem(entry['node']))
        self.table_logs.setItem(row, 3, QTableWidgetItem(entry['msg']))

        # Limit row count to 200
        if self.table_logs.rowCount() > 200:
            self.table_logs.removeRow(0)

        self.table_logs.scrollToBottom()

    def _filter_logs(self):
        choice = self.cb_filter_level.currentText()
        for r in range(self.table_logs.rowCount()):
            lvl = self.table_logs.item(r, 1).text()
            if "SEMUA" in choice:
                self.table_logs.setRowHidden(r, False)
            elif "WARN" in choice:
                self.table_logs.setRowHidden(r, lvl != 'WARN')
            elif "ERROR" in choice:
                self.table_logs.setRowHidden(r, lvl != 'ERROR')

    def _clear_logs_table(self):
        self.table_logs.setRowCount(0)

    def _on_new_incident(self, inc: dict):
        self.lbl_missed_badge.setText(f"Total Missed Loops: {inc['id']}")
        self.lbl_missed_badge.setStyleSheet("background-color: #f38ba8; color: #11111b; padding: 4px 10px; border-radius: 4px; font-weight: bold;")

        row = self.table_incidents.rowCount()
        self.table_incidents.insertRow(row)
        self.table_incidents.setItem(row, 0, QTableWidgetItem(str(inc['id'])))
        self.table_incidents.setItem(row, 1, QTableWidgetItem(inc['time']))
        self.table_incidents.setItem(row, 2, QTableWidgetItem(f"[{inc['node']}] {inc['msg'][:40]}..."))
        causes_str = " | ".join(inc.get('causes', []))
        self.table_incidents.setItem(row, 3, QTableWidgetItem(causes_str))
        self.table_incidents.scrollToBottom()

    # ==================== CALIBRATION & MOTION TEST ====================
    def _on_odom_update(self, odom: dict):
        x = odom['x']
        y = odom['y']
        th_rad = odom['theta']
        th_deg = math.degrees(th_rad)

        self.lbl_odom_x.setText(f"<b>X (m):</b> <font color='#89b4fa' size='+1'>{x:.3f}</font>")
        self.lbl_odom_y.setText(f"<b>Y (m):</b> <font color='#89b4fa' size='+1'>{y:.3f}</font>")
        self.lbl_odom_theta.setText(f"<b>Yaw:</b> <font color='#89b4fa' size='+1'>{th_deg:.1f}° ({th_rad:.3f} rad)</font>")
        self.lbl_odom_vel.setText(f"<b>Kecepatan:</b> Vx: {odom['vx']:.2f} | Vy: {odom['vy']:.2f} | Wz: {odom['wz']:.2f}")

        # Relative distance from tare
        dx = x - self.tare_x
        dy = y - self.tare_y
        dist = math.sqrt(dx * dx + dy * dy)
        self.lbl_rel_distance.setText(f"<b>Jarak Tempuh Sejak Zero:</b> <font color='#a6e3a1' size='+1'>{dist:.3f} m</font>")

    def _tare_software_zero(self):
        with self.ros_node.lock:
            self.tare_x = self.ros_node.latest_odom_stats['x']
            self.tare_y = self.ros_node.latest_odom_stats['y']
        QMessageBox.information(self, "Software Tare", f"Software Zero diset ke X={self.tare_x:.3f}, Y={self.tare_y:.3f}")

    def _trigger_node_odom_reset(self):
        self.ros_node.trigger_reset_odom()
        self.tare_x = 0.0
        self.tare_y = 0.0

    def _run_motion_test(self):
        t_type = self.combo_test_type.currentData()
        target = self.spin_test_target.value()
        spd = self.spin_test_speed.value()

        # Reset progress
        self.progress_motion.setValue(0)
        self.ros_node.start_motion_test(t_type, target, spd)

    def _estop(self):
        self.ros_node.stop_motion_test()
        self.ros_node.send_cmd_vel(0.0, 0.0, 0.0)
        self.lbl_motion_status.setText("🛑 EMERGENCY STOP DIAKTIFKAN.")

    def _on_motion_test_progress(self, current: float, target: float, status_str: str):
        self.lbl_motion_status.setText(f"Status: {status_str}")
        pct = int(min(100.0, max(0.0, (current / target) * 100.0))) if target > 0 else 0
        self.progress_motion.setValue(pct)

    def _copy_last_test_distance(self):
        rec = self.ros_node.last_test_distance_recorded
        if rec > 0.0:
            self.spin_odom_dist.setValue(rec)
        else:
            QMessageBox.information(self, "Info", "Belum ada hasil uji gerak otomatis yang selesai.")

    def _calculate_scale(self):
        cur_scale = self.spin_current_scale.value()
        d_odom = self.spin_odom_dist.value()
        d_act = self.spin_actual_dist.value()

        if d_odom <= 0.0:
            QMessageBox.warning(self, "Error", "Jarak odometri tidak boleh 0.")
            return

        # New Scale = Old Scale * (Actual / Odom)
        new_scale = cur_scale * (d_act / d_odom)
        delta_pct = ((new_scale - cur_scale) / cur_scale) * 100.0
        self.calculated_new_scale = new_scale

        axis = self.combo_axis.currentData()
        self.lbl_calc_result.setText(
            f"<b>Skala Baru ({axis}): <font color='#a6e3a1' size='+2'>{new_scale:.4f}</font></b>"
        )
        self.lbl_calc_delta.setText(f"Koreksi: <b>{delta_pct:+.2f}%</b> (Dari {cur_scale:.4f} ➜ {new_scale:.4f})")

    def _apply_runtime_scale(self):
        if self.calculated_new_scale is None:
            self._calculate_scale()

        axis = self.combo_axis.currentData()
        new_scale = self.calculated_new_scale

        ok, msg = self.ros_node.set_odometry_scale_param(axis, new_scale)
        if ok:
            self.spin_current_scale.setValue(new_scale)
            QMessageBox.information(self, "Berhasil", f"✅ {msg}")
        else:
            QMessageBox.warning(self, "Gagal", f"⚠️ {msg}")

    def _save_scale_to_yaml(self):
        if self.calculated_new_scale is None:
            self._calculate_scale()

        axis = self.combo_axis.currentData()
        new_scale = self.calculated_new_scale

        # File target: nav2_params_paper.yaml
        yaml_path = os.path.join(self.ros_node.ws_root, 'src', 'robot_bringup', 'config', 'nav2_params_paper.yaml')
        if not os.path.exists(yaml_path):
            QMessageBox.warning(self, "Error", f"File tidak ditemukan: {yaml_path}")
            return

        try:
            with open(yaml_path, 'r') as f:
                content = f.read()

            # Replace or ensure parameter in odometry_node if applicable
            # In nav2_params_paper.yaml or odometry_node.py
            # Since scale is in odometry_node.py, also check odometry_node.py
            py_path = os.path.join(self.ros_node.ws_root, 'src', 'robot_base', 'robot_base', 'odometry_node.py')
            if os.path.exists(py_path):
                with open(py_path, 'r') as f:
                    py_content = f.read()
                
                target_str = f"self.declare_parameter('{axis}',"
                if target_str in py_content:
                    import re
                    pattern = rf"self\.declare_parameter\('{axis}',\s*[\d\.]+\)"
                    repl = f"self.declare_parameter('{axis}', {new_scale:.4f})"
                    new_py_content = re.sub(pattern, repl, py_content)
                    with open(py_path, 'w') as f:
                        f.write(new_py_content)

            QMessageBox.information(self, "Tersimpan", f"✅ Skala {axis} = {new_scale:.4f} berhasil disimpan ke file kode!")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Gagal menulis file: {e}")

    # ==================== SAVE SESSION ====================
    def _save_session(self):
        sess_id = self.ros_node.session_id
        json_file = os.path.join(self.ros_node.log_dir, f"session_{sess_id}.json")
        md_file = os.path.join(self.ros_node.log_dir, f"session_{sess_id}_report.md")

        with self.ros_node.lock:
            incidents_copy = list(self.ros_node.incidents)
            history_copy = list(self.ros_node.metrics_history)
            scan_stats = dict(self.ros_node.latest_scan_stats)
            odom_stats = dict(self.ros_node.latest_odom_stats)
            tf_stats = dict(self.ros_node.latest_tf_stats)
            total_missed = self.ros_node.total_missed_loops

        now = datetime.now()
        duration = (now - self.ros_node.session_start_time).total_seconds()

        export_data = {
            'session_id': sess_id,
            'start_time': self.ros_node.session_start_time.isoformat(),
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

            # Generate Markdown report
            with open(md_file, 'w') as f:
                f.write(f"# Laporan Diagnostik Nav2 - Sesi {sess_id}\n\n")
                f.write(f"- **Waktu**: {self.ros_node.session_start_time} s/d {now} ({duration:.1f} detik)\n")
                f.write(f"- **Total Missed Loops**: **{total_missed} kali**\n")
                f.write(f"- **Frekuensi LiDAR (/scan)**: {scan_stats.get('rate_hz', 0.0)} Hz (Delay: {scan_stats.get('delay_ms', 0.0)} ms)\n")
                f.write(f"- **Frekuensi Odometri (/odom)**: {odom_stats.get('rate_hz', 0.0)} Hz\n\n")
                f.write("## Rekaman Insiden:\n")
                if not incidents_copy:
                    f.write("✅ Tidak ada insiden missed-loop selama sesi ini.\n")
                else:
                    for inc in incidents_copy:
                        f.write(f"### Insiden #{inc['id']} - {inc['time']}\n")
                        f.write(f"- Pesan: `{inc['msg']}`\n")
                        f.write(f"- Penyebab: {', '.join(inc.get('causes', []))}\n\n")

            QMessageBox.information(self, "Tersimpan", f"✅ Data Sesi Berhasil Disimpan!\n\nJSON:\n{json_file}\n\nReport:\n{md_file}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Gagal menyimpan data sesi: {e}")


# ==============================================================================
# 5. ENTRYPOINT
# ==============================================================================
def main():
    rclpy.init(args=sys.argv)
    ros_bridge = RosBackendBridge()

    # Spin ROS node in background daemon thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_bridge,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = DiagnosticsMainWindow(ros_bridge)
    window.show()

    exit_code = app.exec_()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
