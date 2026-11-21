#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import math
import sys
import os
import re

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
        self.update_command_config(x, y, yaw)
        
        self.get_logger().info("Selesai! Silakan tekan Ctrl+C.")
        sys.exit(0)

    def save_to_yaml(self, x, y, yaw):
        # Resolusi path absolut ke src/robot_command/waypoints.yaml
        script_dir = os.path.dirname(os.path.abspath(__file__))
        yaml_file = os.path.join(script_dir, 'src', 'robot_command', 'waypoints.yaml')
        
        os.makedirs(os.path.dirname(yaml_file), exist_ok=True)

        data = {}
        if os.path.exists(yaml_file):
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f) or {}

        data[self.room_name] = {'x': x, 'y': y, 'yaw': yaw}

        with open(yaml_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f"Berhasil menyimpan {self.room_name} ke {yaml_file}")

    def update_command_config(self, x, y, yaw):
        """Memperbarui atau menambahkan koordinat waypoint ke src/robot_command/config/command_config.yaml."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, 'src', 'robot_command', 'config', 'command_config.yaml')
        
        if not os.path.exists(config_path):
            self.get_logger().warn(f"File {config_path} tidak ditemukan, melewati update command_config.")
            return

        with open(config_path, 'r', encoding='utf-8') as f:
            content = f.read()

        wp_idx = content.find('\nwaypoints:')
        if wp_idx == -1 and content.startswith('waypoints:'):
            header = ''
            wp_part = content
        elif wp_idx != -1:
            header = content[:wp_idx + 1]
            wp_part = content[wp_idx + 1:]
        else:
            self.get_logger().warn(f"Section 'waypoints:' tidak ditemukan di {config_path}")
            return

        room_pattern = re.compile(
            rf'(^  {re.escape(self.room_name)}:\s*\n)(.*?)(?=(^  [a-zA-Z0-9_]+:|\Z))',
            re.MULTILINE | re.DOTALL
        )
        match = room_pattern.search(wp_part)

        if match:
            bh = match.group(1)
            bb = match.group(2)
            if re.search(r'^\s+x:\s*.*$', bb, re.MULTILINE):
                bb = re.sub(r'^(\s+x:\s*).*$', rf'\g<1>{x}', bb, flags=re.MULTILINE)
            else:
                bb = f'    x: {x}\n' + bb

            if re.search(r'^\s+y:\s*.*$', bb, re.MULTILINE):
                bb = re.sub(r'^(\s+y:\s*).*$', rf'\g<1>{y}', bb, flags=re.MULTILINE)
            else:
                bb = f'    y: {y}\n' + bb

            if re.search(r'^\s+yaw:\s*.*$', bb, re.MULTILINE):
                bb = re.sub(r'^(\s+yaw:\s*).*$', rf'\g<1>{yaw}', bb, flags=re.MULTILINE)
            else:
                bb = f'    yaw: {yaw}\n' + bb

            new_wp = wp_part[:match.start()] + bh + bb + wp_part[match.end():]
            self.get_logger().info(f"Berhasil memperbarui {self.room_name} di {config_path}")
        else:
            disp_name = self.room_name.replace('_', ' ').title()
            clean_alias = self.room_name.replace('_', ' ').lower()
            new_entry = (
                f"\n  {self.room_name}:\n"
                f"    x: {x}\n"
                f"    y: {y}\n"
                f"    yaw: {yaw}\n"
                f"    name: \"{disp_name}\"\n"
                f"    aliases: [\"{clean_alias}\"]\n"
            )
            new_wp = wp_part.rstrip() + "\n" + new_entry
            self.get_logger().info(f"Berhasil menambahkan entri baru {self.room_name} ke {config_path}")

        new_content = header + new_wp
        with open(config_path, 'w', encoding='utf-8') as f:
            f.write(new_content)

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
