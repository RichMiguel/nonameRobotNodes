#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import os
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
        
        # Inisialisasi TTY / Terminal
        self.tty_file = None
        self.tty_fd = None
        self.settings = None
        
        # Jika stdin adalah TTY langsung (misal saat ros2 run)
        if sys.stdin.isatty():
            self.tty_fd = sys.stdin.fileno()
        else:
            # Jika dijalankan melalui ros2 launch, sys.stdin di-pipe; gunakan /dev/tty
            try:
                self.tty_file = open('/dev/tty', 'r')
                self.tty_fd = self.tty_file.fileno()
            except Exception as e:
                self.get_logger().warn(f"Tidak dapat mengakses terminal (/dev/tty): {e}")
                self.tty_fd = None
                
        if self.tty_fd is not None:
            try:
                self.settings = termios.tcgetattr(self.tty_fd)
            except Exception as e:
                self.get_logger().warn(f"Gagal membaca atribut terminal: {e}")
                self.settings = None
        
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

    def restore_terminal(self):
        if self.tty_fd is not None and self.settings is not None:
            try:
                termios.tcsetattr(self.tty_fd, termios.TCSANOW, self.settings)
            except Exception:
                pass
        if self.tty_file is not None:
            try:
                self.tty_file.close()
            except Exception:
                pass
            self.tty_file = None

    def get_key(self):
        if self.tty_fd is None:
            return ''
        key = ''
        try:
            tty.setraw(self.tty_fd)
            # Tunggu maksimal 0.05 detik untuk input keyboard
            rlist, _, _ = select.select([self.tty_fd], [], [], 0.05)
            if rlist:
                key = os.read(self.tty_fd, 1).decode('utf-8', errors='ignore')
        except Exception:
            key = ''
        finally:
            if self.tty_fd is not None and self.settings is not None:
                termios.tcsetattr(self.tty_fd, termios.TCSANOW, self.settings)
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
            self.restore_terminal()
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
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
