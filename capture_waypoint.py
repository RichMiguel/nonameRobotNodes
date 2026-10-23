#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import math
import sys
import os

class WaypointCapturer(Node):
    def __init__(self, room_name):
        super().__init__('waypoint_capturer')
        self.room_name = room_name
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.listener_callback,
            10)
        self.get_logger().info(f"Menunggu Anda mengklik '2D Goal Pose' di RViz2 untuk '{self.room_name}'...")

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

    def listener_callback(self, msg):
        x = round(msg.pose.position.x, 3)
        y = round(msg.pose.position.y, 3)
        
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = round(self.euler_from_quaternion(qx, qy, qz, qw), 3)

        self.get_logger().info(f"Koordinat ditangkap -> X: {x}, Y: {y}, Yaw: {yaw}")
        self.save_to_yaml(x, y, yaw)
        
        self.get_logger().info("Selesai! Silakan tekan Ctrl+C.")
        sys.exit(0)

    def save_to_yaml(self, x, y, yaw):
        # Gunakan path relatif ke package atau tempat skrip dieksekusi
        yaml_file = 'src/robot_voice_nav/waypoints.yaml'
        
        data = {}
        if os.path.exists(yaml_file):
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f) or {}

        data[self.room_name] = {'x': x, 'y': y, 'yaw': yaw}

        with open(yaml_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f"Berhasil menyimpan {self.room_name} ke {yaml_file}")

def main(args=None):
    if len(sys.argv) < 2:
        print("Penggunaan: python3 capture_waypoint.py <nama_ruangan>")
        print("Contoh: python3 capture_waypoint.py ruang_icu")
        sys.exit(1)

    rclpy.init(args=args)
    node = WaypointCapturer(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
