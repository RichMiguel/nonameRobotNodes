#!/usr/bin/env python3
# ==============================================================================
# Robot Command Coordinator Node (robot_command)
# Menerima perintah suara & dashboard, mengkoordinasikan Nav2,
# mendeteksi rintangan & replanning, serta mengirim feedback real-time.
# ==============================================================================

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose

import socket
import select
import threading
import json
import yaml
import os
import math
import time
from typing import Dict, Any, Optional

class CommandCoordinatorNode(Node):
    def __init__(self):
        super().__init__('robot_command_node')
        
        # 1. Deklarasi Parameter ROS 2
        self.declare_parameter('config_file', '')
        self.declare_parameter('tcp_port', 5000)
        
        # 2. Muat Konfigurasi YAML
        self.config = {}
        self.waypoints = {}
        self.feedback_templates = {}
        self.load_configuration()
        
        # 3. State Internal Robot
        self.state = "IDLE"  # IDLE, NAVIGATING, OBSTACLE_BLOCKED, REPLANNING, ARRIVED, CANCELLED, ERROR
        self.current_goal_name: Optional[str] = None
        self.current_goal_target: Optional[Dict[str, float]] = None
        self.current_goal_handle = None
        self.active_nav2_goal_id: Optional[str] = None
        
        # Variabel Odometri & Tracking
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.current_twist = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        self.distance_remaining = 0.0
        self.estimated_time_remaining = 0.0
        
        # Variabel Safety (LiDAR)
        self.obstacle_detected = False
        self.last_obstacle_dist = 999.0
        self.last_obstacle_alert_time = 0.0
        
        # Variabel Path & Replanning
        self.last_path_length = 0
        self.last_path_points_count = 0
        self.is_replanning = False
        self.last_replan_time = 0.0
        
        # 4. Inisialisasi ROS 2 Publishers
        self.goal_pub = self.create_publisher(PoseStamped, self.config['navigation']['goal_pose_topic'], 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, '/robot_command/feedback', 10)
        self.voice_feedback_pub = self.create_publisher(String, '/robot_command/voice_feedback', 10)
        self.status_pub = self.create_publisher(String, '/robot_command/status', 10)
        
        # 5. Inisialisasi ROS 2 Subscribers
        self.create_subscription(String, '/robot_command/input', self.command_topic_callback, 10)
        
        scan_topic = self.config['navigation'].get('scan_topic', '/scan')
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        
        plan_topic = self.config['navigation'].get('plan_topic', '/plan')
        self.create_subscription(Path, plan_topic, self.plan_callback, 10)
        
        odom_topic = self.config['navigation'].get('odom_topic', '/odom')
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        
        # Pantau status action server Nav2
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav2_status_callback, 10)
        
        # 6. Action Client ke Nav2
        action_server = self.config['navigation'].get('action_server', 'navigate_to_pose')
        self.nav2_client = ActionClient(self, NavigateToPose, action_server)
        
        # 7. Timer Periodik Status Publish (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_periodic_status)
        
        # 8. Setup TCP Server (Multi-client broadcast)
        server_cfg = self.config.get('server', {})
        self.server_host = server_cfg.get('host', '0.0.0.0')
        param_port = self.get_parameter('tcp_port').value
        self.server_port = int(param_port if param_port else server_cfg.get('port', 5000))
        
        self.tcp_clients = []
        self.tcp_clients_lock = threading.Lock()
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.server_host, self.server_port))
        self.server_socket.listen(5)
        
        self.get_logger().info(f"🚀 Robot Command Coordinator aktif! TCP Server siap di {self.server_host}:{self.server_port}")
        
        # Jalankan server TCP dalam background thread
        self.server_thread = threading.Thread(target=self.tcp_server_loop, daemon=True)
        self.server_thread.start()

    # ==========================================================================
    # KONFIGURASI & WAYPOINTS
    # ==========================================================================
    def load_configuration(self):
        """Memuat command_config.yaml atau fallback ke waypoints.yaml."""
        from ament_index_python.packages import get_package_share_directory
        
        default_config = {
            'server': {'host': '0.0.0.0', 'port': 5000, 'broadcast_feedback': True},
            'safety': {
                'enable_obstacle_detection': True,
                'warning_distance': 0.55,
                'critical_distance': 0.30,
                'front_angle_min_deg': -50.0,
                'front_angle_max_deg': 50.0,
                'throttle_feedback_sec': 2.5
            },
            'navigation': {
                'action_server': 'navigate_to_pose',
                'goal_pose_topic': '/goal_pose',
                'scan_topic': '/scan',
                'plan_topic': '/plan',
                'odom_topic': '/odom',
                'replan_detection': True,
                'replan_min_points_diff': 5,
                'goal_timeout_sec': 180.0
            },
            'feedback_templates': {
                'GOAL_ACCEPTED': 'Perintah navigasi ke {location} diterima. Robot mulai bergerak.',
                'NAVIGATING': 'Robot sedang bergerak menuju {location}.',
                'GOAL_REACHED': 'Robot sudah sampai di {location}.',
                'OBSTACLE_WARNING': 'Peringatan! Terdeteksi rintangan di depan robot pada jarak {distance:.2f} meter.',
                'OBSTACLE_CLEARED': 'Rintangan di depan telah bersih. Robot melanjutkan perjalanan.',
                'REPLANNING_PATH': 'Jalur terhalang. Sedang menghitung lintasan baru menuju {location}.',
                'GOAL_CANCELLED': 'Perintah navigasi dibatalkan. Robot berhenti.',
                'GOAL_ABORTED': 'Gagal mencapai {location}. Rute terhalang total.',
                'EMERGENCY_STOP': 'Pemberhentian darurat diaktifkan.',
                'UNKNOWN_COMMAND': 'Perintah tidak dikenali oleh sistem.',
                'LOCATION_NOT_FOUND': 'Lokasi {location} tidak ditemukan dalam daftar waypoint.'
            },
            'waypoints': {}
        }
        
        cfg_path = self.get_parameter('config_file').value
        if not cfg_path:
            try:
                pkg_dir = get_package_share_directory('robot_command')
                primary_cfg = os.path.join(pkg_dir, 'config', 'command_config.yaml')
                legacy_cfg = os.path.join(pkg_dir, 'waypoints.yaml')
                cfg_path = primary_cfg if os.path.exists(primary_cfg) else legacy_cfg
            except Exception:
                cfg_path = 'config/command_config.yaml'

        if os.path.exists(cfg_path):
            try:
                with open(cfg_path, 'r', encoding='utf-8') as f:
                    loaded = yaml.safe_load(f)
                    if isinstance(loaded, dict):
                        if 'waypoints' in loaded:
                            default_config.update(loaded)
                        else:
                            # Kompatibilitas format waypoints.yaml lama
                            default_config['waypoints'] = loaded
                self.get_logger().info(f"Berhasil memuat konfigurasi dari: {cfg_path}")
            except Exception as e:
                self.get_logger().warn(f"Gagal membaca {cfg_path}: {e}. Memakai konfigurasi default.")
        else:
            self.get_logger().warn(f"File config {cfg_path} tidak ditemukan, memakai pengaturan default.")
            
        self.config = default_config
        self.waypoints = self.config.get('waypoints', {})
        self.feedback_templates = self.config.get('feedback_templates', {})

    def resolve_location(self, query: str) -> Optional[Dict[str, Any]]:
        """Mencari waypoint berdasarkan key ID, nama, atau alias suara."""
        if not query:
            return None
        q = query.lower().strip().replace(" ", "_")
        
        # 1. Cek kecocokan key langsung
        if q in self.waypoints:
            wp = self.waypoints[q].copy()
            wp['id'] = q
            return wp
            
        # 2. Cek kecocokan alternatif tanpa underscore / alias
        q_clean = query.lower().strip()
        for key, data in self.waypoints.items():
            if key.replace("_", " ") == q_clean:
                wp = data.copy()
                wp['id'] = key
                return wp
            name = str(data.get('name', '')).lower()
            if name == q_clean or q_clean in name:
                wp = data.copy()
                wp['id'] = key
                return wp
            for alias in data.get('aliases', []):
                if alias.lower() == q_clean:
                    wp = data.copy()
                    wp['id'] = key
                    return wp
        return None

    # ==========================================================================
    # SISTEM FEEDBACK (Topik ROS 2, TTS Voice, & Broadcast Socket)
    # ==========================================================================
    def emit_feedback(self, event_type: str, custom_message: Optional[str] = None, data: Optional[Dict[str, Any]] = None):
        """Membuat dan mendistribusikan feedback terstruktur ke seluruh sistem."""
        data = data or {}
        location_label = data.get('name') or data.get('location') or self.current_goal_name or "tujuan"
        distance = data.get('distance', self.last_obstacle_dist)
        
        # Format teks natural dari template
        template = self.feedback_templates.get(event_type, custom_message or event_type)
        try:
            speech_text = template.format(location=location_label, distance=distance)
        except Exception:
            speech_text = custom_message or template

        feedback_payload = {
            "type": event_type,
            "state": self.state,
            "message": speech_text,
            "timestamp": time.time(),
            "data": data
        }
        
        json_str = json.dumps(feedback_payload, ensure_ascii=False)
        
        # 1. Publish JSON ke topik feedback
        fb_msg = String()
        fb_msg.data = json_str
        self.feedback_pub.publish(fb_msg)
        
        # 2. Publish kalimat natural untuk Text-to-Speech (TTS)
        voice_msg = String()
        voice_msg.data = speech_text
        self.voice_feedback_pub.publish(voice_msg)
        
        self.get_logger().info(f"[FEEDBACK] [{event_type}] {speech_text}")
        
        # 3. Broadcast ke seluruh socket client (Dashboard / GUI / Voice Client)
        self.broadcast_tcp_message(json_str)

    def publish_periodic_status(self):
        """Publish status telemetri lengkap robot secara periodik."""
        status_payload = {
            "type": "STATUS_UPDATE",
            "state": self.state,
            "current_goal": self.current_goal_name,
            "pose": self.current_pose,
            "twist": self.current_twist,
            "distance_remaining": self.distance_remaining,
            "obstacle_detected": self.obstacle_detected,
            "obstacle_distance": self.last_obstacle_dist if self.obstacle_detected else None,
            "timestamp": time.time()
        }
        status_msg = String()
        status_msg.data = json.dumps(status_payload, ensure_ascii=False)
        self.status_pub.publish(status_msg)

    # ==========================================================================
    # DETEKSI RINTANGAN & REPLANNING (Safety Monitoring)
    # ==========================================================================
    def scan_callback(self, msg: LaserScan):
        """Analisis laser scan real-time untuk mendeteksi rintangan di sektor depan robot."""
        safety_cfg = self.config.get('safety', {})
        if not safety_cfg.get('enable_obstacle_detection', True):
            return
            
        warn_dist = safety_cfg.get('warning_distance', 0.55)
        crit_dist = safety_cfg.get('critical_distance', 0.30)
        min_angle = math.radians(safety_cfg.get('front_angle_min_deg', -50.0))
        max_angle = math.radians(safety_cfg.get('front_angle_max_deg', 50.0))
        throttle_sec = safety_cfg.get('throttle_feedback_sec', 2.5)
        
        front_ranges = []
        angle = msg.angle_min
        for r in msg.ranges:
            if min_angle <= angle <= max_angle:
                if msg.range_min < r < msg.range_max:
                    front_ranges.append(r)
            angle += msg.angle_increment

        if not front_ranges:
            min_front = 999.0
        else:
            min_front = min(front_ranges)
            
        self.last_obstacle_dist = min_front
        current_time = time.time()

        # Deteksi Rintangan Aktif
        if min_front < warn_dist:
            if not self.obstacle_detected:
                self.obstacle_detected = True
                if self.state == "NAVIGATING":
                    self.state = "OBSTACLE_BLOCKED"
                self.emit_feedback("OBSTACLE_WARNING", data={
                    "distance": min_front,
                    "is_critical": (min_front < crit_dist)
                })
                self.last_obstacle_alert_time = current_time
            elif (current_time - self.last_obstacle_alert_time) > throttle_sec:
                # Refresh peringatan jika rintangan masih bertahan
                self.emit_feedback("OBSTACLE_WARNING", data={
                    "distance": min_front,
                    "is_critical": (min_front < crit_dist)
                })
                self.last_obstacle_alert_time = current_time
        else:
            if self.obstacle_detected:
                self.obstacle_detected = False
                if self.state == "OBSTACLE_BLOCKED":
                    self.state = "NAVIGATING"
                self.emit_feedback("OBSTACLE_CLEARED", data={"distance": min_front})

    def plan_callback(self, msg: Path):
        """Mendeteksi apakah Nav2 sedang menghitung ulang lintasan (Replanning)."""
        if not self.config['navigation'].get('replan_detection', True):
            return
            
        points_count = len(msg.poses)
        if points_count == 0:
            return
            
        min_diff = self.config['navigation'].get('replan_min_points_diff', 5)
        now = time.time()
        
        # Jika robot sedang navigasi dan struktur path berubah signifikan
        if self.state in ["NAVIGATING", "OBSTACLE_BLOCKED"] and self.last_path_points_count > 0:
            if abs(points_count - self.last_path_points_count) >= min_diff and (now - self.last_replan_time) > 3.0:
                self.is_replanning = True
                self.last_replan_time = now
                self.emit_feedback("REPLANNING_PATH", data={
                    "location": self.current_goal_name,
                    "path_points": points_count
                })
                
        self.last_path_points_count = points_count

    def odom_callback(self, msg: Odometry):
        """Simpan odometri posisi robot terkini."""
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_pose['yaw'] = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_twist['vx'] = msg.twist.twist.linear.x
        self.current_twist['vy'] = msg.twist.twist.linear.y
        self.current_twist['wz'] = msg.twist.twist.angular.z

    # ==========================================================================
    # LOGIKA EKSEKUSI PERINTAH NAVIGASI
    # ==========================================================================
    def navigate_to_target(self, x: float, y: float, yaw: float, name: str = "Tujuan"):
        """Mengirim target koordinat ke Nav2 & menyetel state robot."""
        self.current_goal_name = name
        self.current_goal_target = {'x': x, 'y': y, 'yaw': yaw}
        self.state = "NAVIGATING"
        
        # Buat pesan PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 1. Publish ke topik /goal_pose (Kompatibilitas FSM & RViz)
        self.goal_pub.publish(pose)
        
        # 2. Kirim via Action Client jika server Nav2 aktif
        if self.nav2_client.wait_for_server(timeout_sec=0.5):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            send_goal_future = self.nav2_client.send_goal_async(
                goal_msg, 
                feedback_callback=self.nav2_feedback_callback
            )
            send_goal_future.add_done_callback(self.nav2_goal_response_callback)
        else:
            self.get_logger().info("Nav2 Action Server belum siap, target dipublish via /goal_pose.")
            
        self.emit_feedback("GOAL_ACCEPTED", data={
            "location": name,
            "x": x,
            "y": y,
            "yaw": yaw
        })

    def nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Target '{self.current_goal_name}' ditolak oleh Nav2!")
            self.state = "ERROR"
            self.emit_feedback("GOAL_ABORTED", data={"location": self.current_goal_name, "reason": "Goal rejected"})
            return

        self.current_goal_handle = goal_handle
        self.active_nav2_goal_id = str(goal_handle.goal_id)
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.nav2_result_callback)

    def nav2_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.distance_remaining = getattr(fb, 'distance_remaining', 0.0)
        self.estimated_time_remaining = getattr(fb, 'estimated_time_remaining', 0.0)

    def nav2_result_callback(self, future):
        try:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.state = "ARRIVED"
                self.emit_feedback("GOAL_REACHED", data={"location": self.current_goal_name})
            elif status == GoalStatus.STATUS_CANCELED:
                self.state = "CANCELLED"
                self.emit_feedback("GOAL_CANCELLED", data={"location": self.current_goal_name})
            elif status == GoalStatus.STATUS_ABORTED:
                self.state = "ERROR"
                self.emit_feedback("GOAL_ABORTED", data={"location": self.current_goal_name})
        except Exception as e:
            self.get_logger().error(f"Error pada callback result Nav2: {e}")
        finally:
            self.current_goal_handle = None

    def nav2_status_callback(self, msg: GoalStatusArray):
        """Pantau status goal Nav2 jika dikirim dari node luar (misal RViz atau FSM)."""
        if not msg.status_list or self.state not in ["NAVIGATING", "OBSTACLE_BLOCKED"]:
            return
            
        latest_status = msg.status_list[-1]
        status_code = latest_status.status
        
        if status_code == GoalStatus.STATUS_SUCCEEDED and self.state != "ARRIVED":
            self.state = "ARRIVED"
            self.emit_feedback("GOAL_REACHED", data={"location": self.current_goal_name or "Tujuan"})
        elif status_code == GoalStatus.STATUS_CANCELED and self.state != "CANCELLED":
            self.state = "CANCELLED"
            self.emit_feedback("GOAL_CANCELLED", data={"location": self.current_goal_name or "Tujuan"})
        elif status_code == GoalStatus.STATUS_ABORTED and self.state != "ERROR":
            self.state = "ERROR"
            self.emit_feedback("GOAL_ABORTED", data={"location": self.current_goal_name or "Tujuan"})

    def stop_robot(self, emergency: bool = False):
        """Hentikan gerak robot & batalkan goal aktif di Nav2."""
        self.state = "IDLE"
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None
            
        # Kirim perintah kecepatan 0 ke motor
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        event = "EMERGENCY_STOP" if emergency else "GOAL_CANCELLED"
        self.emit_feedback(event, data={"reason": "Manual Stop / Emergency"})

    # ==========================================================================
    # PARSER PERINTAH (TCP Socket, Topic, & API)
    # ==========================================================================
    def execute_command_dict(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """Eksekutor terpusat untuk semua format perintah JSON."""
        intent = str(cmd.get("intent") or cmd.get("command") or "").upper().strip()
        slots = cmd.get("slots") or {}
        
        # Ekstrak slot/field opsional
        location = slots.get("location") or cmd.get("location") or cmd.get("target") or ""
        x = slots.get("x") or cmd.get("x")
        y = slots.get("y") or cmd.get("y")
        yaw = slots.get("yaw") or cmd.get("yaw", 0.0)

        # 1. NAVIGATE / GO_TO
        if intent in ["NAVIGATE", "GO_TO", "NAV"]:
            if location:
                wp = self.resolve_location(str(location))
                if wp:
                    self.navigate_to_target(wp['x'], wp['y'], wp['yaw'], wp.get('name', location))
                    return {"success": True, "status": "ACCEPTED", "message": f"Menuju {wp.get('name', location)}", "data": wp}
                else:
                    self.emit_feedback("LOCATION_NOT_FOUND", data={"location": location})
                    return {"success": False, "status": "ERROR", "message": f"Lokasi '{location}' tidak ditemukan."}
            elif x is not None and y is not None:
                self.navigate_to_target(float(x), float(y), float(yaw), f"Koordinat ({x:.2f}, {y:.2f})")
                return {"success": True, "status": "ACCEPTED", "message": f"Menuju koordinat ({x}, {y})"}
            else:
                return {"success": False, "status": "ERROR", "message": "Parameter lokasi atau koordinat (x, y) wajib diisi."}

        # 2. STOP / CANCEL / EMERGENCY
        elif intent in ["STOP", "CANCEL", "EMERGENCY", "PAUSE"]:
            is_emergency = (intent == "EMERGENCY")
            self.stop_robot(emergency=is_emergency)
            return {"success": True, "status": "STOPPED", "message": "Robot berhasil dihentikan."}

        # 3. GET_STATUS
        elif intent in ["STATUS", "GET_STATUS"]:
            return {
                "success": True,
                "status": self.state,
                "data": {
                    "state": self.state,
                    "current_goal": self.current_goal_name,
                    "pose": self.current_pose,
                    "distance_remaining": self.distance_remaining,
                    "obstacle_detected": self.obstacle_detected,
                    "obstacle_distance": self.last_obstacle_dist if self.obstacle_detected else None
                }
            }

        # 4. GET_WAYPOINTS
        elif intent in ["GET_WAYPOINTS", "LIST_LOCATIONS"]:
            return {"success": True, "waypoints": self.waypoints}

        # 5. GO_BACK / HOME
        elif intent in ["GO_BACK", "HOME"]:
            wp = self.resolve_location("home") or {"x": 0.0, "y": 0.0, "yaw": 0.0, "name": "Home"}
            self.navigate_to_target(wp['x'], wp['y'], wp['yaw'], wp.get('name', "Home"))
            return {"success": True, "status": "ACCEPTED", "message": "Kembali ke posisi Home"}

        # 6. UNKNOWN
        else:
            self.emit_feedback("UNKNOWN_COMMAND", data={"raw_intent": intent})
            return {"success": False, "status": "UNKNOWN", "message": f"Perintah '{intent}' tidak dikenali."}

    def command_topic_callback(self, msg: String):
        """Menerima perintah melalui topik ROS 2 /robot_command/input."""
        try:
            data = json.loads(msg.data)
            self.execute_command_dict(data)
        except Exception as e:
            self.get_logger().error(f"Error parsing JSON dari topik /robot_command/input: {e}")

    # ==========================================================================
    # TCP SERVER LOOP (Mendukung Voice Client comand-inst & Dashboard)
    # ==========================================================================
    def tcp_server_loop(self):
        """Loop multi-client socket untuk menerima perintah dan streaming feedback."""
        while rclpy.ok():
            try:
                conn, addr = self.server_socket.accept()
                with self.tcp_clients_lock:
                    self.tcp_clients.append(conn)
                
                # Tangani setiap client di thread mandiri
                t = threading.Thread(target=self.handle_tcp_client, args=(conn, addr), daemon=True)
                t.start()
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f"Error TCP accept: {e}")

    def handle_tcp_client(self, conn: socket.socket, addr):
        self.get_logger().info(f"Client terhubung dari: {addr}")
        conn.settimeout(None)
        
        try:
            while rclpy.ok():
                data = conn.recv(4096)
                if not data:
                    break
                text = data.decode('utf-8').strip()
                if not text:
                    continue
                    
                # Parsing satu atau beberapa JSON command
                try:
                    cmd_json = json.loads(text)
                    response = self.execute_command_dict(cmd_json)
                    response_bytes = (json.dumps(response, ensure_ascii=False) + "\n").encode('utf-8')
                    conn.sendall(response_bytes)
                except json.JSONDecodeError:
                    err_resp = {"success": False, "error": "Invalid JSON string"}
                    conn.sendall((json.dumps(err_resp) + "\n").encode('utf-8'))
        except Exception as e:
            pass
        finally:
            with self.tcp_clients_lock:
                if conn in self.tcp_clients:
                    self.tcp_clients.remove(conn)
            try:
                conn.close()
            except Exception:
                pass
            self.get_logger().info(f"Koneksi dari {addr} ditutup.")

    def broadcast_tcp_message(self, json_str: str):
        """Kirim pesan JSON ke semua client socket yang sedang terhubung."""
        payload = (json_str + "\n").encode('utf-8')
        with self.tcp_clients_lock:
            disconnected = []
            for client in self.tcp_clients:
                try:
                    client.sendall(payload)
                except Exception:
                    disconnected.append(client)
            for d in disconnected:
                if d in self.tcp_clients:
                    self.tcp_clients.remove(d)

def main(args=None):
    rclpy.init(args=args)
    node = CommandCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
