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
        
        # Deklarasi parameter kecepatan agar bisa diatur dari launch file
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('strafe_speed', 0.3)
        
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.strafe_speed = float(self.get_parameter('strafe_speed').value)
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        print("\n" + "=" * 58)
        print("  🎮 TELEOP WASD CONTROLLER - NONAME ROBOT (X-DRIVE)")
        print("=" * 58)
        print("  Kontrol Gerak (Tahan tombol, lepas untuk berhenti):")
        print("    [W] Maju               (+Vx)")
        print("    [S] Mundur             (-Vx)")
        print("    [A] Putar Kiri         (+Wz CCW)")
        print("    [D] Putar Kanan        (-Wz CW)")
        print("    [Q] Geser / Strafe Kiri (+Vy)")
        print("    [E] Geser / Strafe Kanan(-Vy)")
        print("    [X] atau [Spasi]       Berhenti (Stop)")
        print("    [Ctrl+C]               Keluar")
        print("-" * 58)
        print(f"  Kecepatan: Linear={self.linear_speed:.2f} m/s | Strafe={self.strafe_speed:.2f} m/s | Angular={self.angular_speed:.2f} rad/s")
        print("=" * 58 + "\n")
        
        # Timer loop pembacaan keyboard (20 Hz / 50 ms)
        self.timer = self.create_timer(0.05, self.read_keys)

    def get_key(self):
        try:
            tty.setraw(sys.stdin.fileno())
            # Tunggu maksimal 0.05 detik untuk input keyboard
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def read_keys(self):
        key = self.get_key()
        k = key.lower()
        
        twist = Twist()
        
        if k == 'w':
            twist.linear.x = self.linear_speed
        elif k == 's':
            twist.linear.x = -self.linear_speed
        elif k == 'a':
            twist.angular.z = self.angular_speed
        elif k == 'd':
            twist.angular.z = -self.angular_speed
        elif k == 'q':
            twist.linear.y = self.strafe_speed
        elif k == 'e':
            twist.linear.y = -self.strafe_speed
        elif k == 'x' or k == ' ':
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
        elif key == '\x03': # Ctrl+C
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        else:
            # Jika tidak ada tombol yang ditekan, kirim nol untuk berhenti
            twist.linear.x = 0.0
            twist.linear.y = 0.0
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
