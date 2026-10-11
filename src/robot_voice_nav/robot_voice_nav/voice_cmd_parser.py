import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import json
import threading
import yaml
import os

class VoiceCmdParserNode(Node):
    def __init__(self):
        super().__init__('voice_cmd_parser_node')
        
        # Inisialisasi publisher untuk goal_pose Nav2
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Load waypoints
        self.waypoints = {}
        self.load_waypoints()
        
        # Setup TCP Server
        self.server_host = '0.0.0.0'
        self.server_port = 5000
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.server_host, self.server_port))
        self.server_socket.listen(5)
        
        self.get_logger().info(f"Voice Command Parser Node telah aktif. Listening TCP di {self.server_host}:{self.server_port}")
        
        # Jalankan server dalam thread terpisah agar tidak memblokir spin ROS 2
        self.server_thread = threading.Thread(target=self.tcp_server_loop, daemon=True)
        self.server_thread.start()

    def load_waypoints(self):
        from ament_index_python.packages import get_package_share_directory
        try:
            package_share_directory = get_package_share_directory('robot_voice_nav')
            waypoints_file = os.path.join(package_share_directory, 'waypoints.yaml')
            
            if os.path.exists(waypoints_file):
                with open(waypoints_file, 'r') as f:
                    self.waypoints = yaml.safe_load(f)
                self.get_logger().info(f"Berhasil memuat waypoints dari {waypoints_file}.")
            else:
                raise FileNotFoundError
        except Exception as e:
            self.get_logger().error(f"Gagal memuat waypoints: {e}")
            # Fallback mock waypoints (jika file belum diset dengan benar di runtime)
            self.waypoints = {
                "ruang_icu": {"x": 5.0, "y": 2.0, "yaw": 1.57},
                "kamar_203": {"x": 10.0, "y": -3.5, "yaw": 0.0},
                "lobi": {"x": 0.0, "y": 0.0, "yaw": -1.57}
            }
            self.get_logger().warn(f"Menggunakan waypoints default/fallback.")

    def tcp_server_loop(self):
        while rclpy.ok():
            try:
                conn, addr = self.server_socket.accept()
                with conn:
                    self.get_logger().info(f"Menerima koneksi dari {addr}")
                    data = conn.recv(4096)
                    if data:
                        self.process_command(data.decode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Error pada TCP Server: {e}")

    def process_command(self, json_str):
        try:
            command = json.loads(json_str)
            intent = command.get("intent", "")
            slots = command.get("slots", {})
            
            self.get_logger().info(f"Command diterima: Intent={intent}, Slots={slots}")
            
            if intent == "NAVIGATE":
                location = slots.get("location", "").lower().replace(" ", "_")
                
                if location in self.waypoints:
                    wp = self.waypoints[location]
                    self.publish_goal(wp['x'], wp['y'], wp['yaw'])
                else:
                    self.get_logger().warn(f"Lokasi '{location}' tidak ditemukan di waypoints!")
            
            elif intent == "STOP":
                self.get_logger().info("Perintah STOP diterima! (Bisa implementasikan cancel goal Nav2)")
                # Implementasi stop/cancel Nav2
                
        except json.JSONDecodeError:
            self.get_logger().error("Gagal mendecode JSON dari socket.")
            
    def publish_goal(self, x, y, yaw):
        import math
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        
        # Konversi Yaw (radian) ke Quaternion untuk 2D
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_pub.publish(msg)
        self.get_logger().info(f"Goal_pose di-publish: x={x}, y={y}, yaw={yaw}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
