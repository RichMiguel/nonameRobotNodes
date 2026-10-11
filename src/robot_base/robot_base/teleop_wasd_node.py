#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class TeleopWASD(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("Teleop WASD Node Started!")
        self.get_logger().info("Gunakan tombol W/A/S/D untuk bergerak. Lepas untuk berhenti.")
        self.get_logger().info("Tekan Ctrl+C untuk keluar.")
        
        # Timer loop for reading keys
        self.timer = self.create_timer(0.05, self.read_keys)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # Tunggu maksimal 0.1 detik untuk input keyboard
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def read_keys(self):
        key = self.get_key()
        
        twist = Twist()
        
        if key == 'w':
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        elif key == 's':
            twist.linear.x = -self.linear_speed
            twist.angular.z = 0.0
        elif key == 'a':
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        elif key == 'd':
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed
        elif key == '\x03': # Ctrl+C
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        else:
            # Jika tidak ada tombol atau tombol salah, berhenti
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopWASD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
