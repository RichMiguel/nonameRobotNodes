#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from geometry_msgs.msg import Twist, TwistStamped

# Konstanta Protokol (ROS2 -> Arduino)
TX_HEADER_1 = 0xAA
TX_HEADER_2 = 0x55
CMD_SET_VELOCITY = 0x01

# Konstanta Protokol (Arduino -> ROS2)
RX_HEADER_1 = 0xBB
RX_HEADER_2 = 0x66
CMD_ODOMETRY = 0x01 # Arduino mengirim feedback menggunakan ID 0x01 (CMD_SET_VELOCITY)

# Faktor Pengali (Multiplier) untuk konversi Float (m/s) ke Int16 (misal mm/s)
# Karena int16 tidak bisa menampung pecahan, kita kalikan sebelum dikirim.
# Jika di Arduino Anda menggunakan pengali yang berbeda (misal 100.0), silakan ubah ini.
VEL_MULTIPLIER = 1000.0

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info(f"Berhasil terkoneksi ke Arduino di {port} dengan baudrate {baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Gagal koneksi ke Arduino: {e}")
            self.serial_port = None

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel_smoothed', self.cmd_vel_callback, 10)
        
        # Publishers
        # Kita mem-publish TwistStamped karena hanya menerima Vx, Vy, Wz dari Arduino.
        # Nanti 'odometry_node' kustom akan men-subscribe topik ini untuk menghitung/integrasi X, Y, Theta
        # dan mem-publish nav_msgs/Odometry beserta TF.
        self.raw_vel_pub = self.create_publisher(TwistStamped, 'raw_odom_vel', 10)
        
        self.create_timer(0.02, self.read_serial_callback)

    def calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def cmd_vel_callback(self, msg: Twist):
        if self.serial_port is None or not self.serial_port.is_open:
            return
            
        # Konversi float ke int16 dengan multiplier
        vx = int(msg.linear.x * VEL_MULTIPLIER)
        vy = int(msg.linear.y * VEL_MULTIPLIER)
        wz = int(msg.angular.z * VEL_MULTIPLIER)
        
        # Batasi nilai agar tetap berada dalam jangkauan int16 (-32768 s/d 32767)
        vx = max(min(vx, 32767), -32768)
        vy = max(min(vy, 32767), -32768)
        wz = max(min(wz, 32767), -32768)
        
        # Packing 3 int16 (6 byte) format little-endian ('<hhh')
        payload = struct.pack('<hhh', vx, vy, wz)
        frame_without_checksum = bytes([TX_HEADER_1, TX_HEADER_2, CMD_SET_VELOCITY]) + payload
        checksum = self.calculate_checksum(frame_without_checksum)
        full_frame = frame_without_checksum + bytes([checksum])
        
        self.serial_port.write(full_frame)

    def read_serial_callback(self):
        if self.serial_port is None or not self.serial_port.is_open:
            return
            
        while self.serial_port.in_waiting >= 2:
            # Cek Header
            h1 = self.serial_port.read(1)
            if h1[0] != RX_HEADER_1:
                continue
                
            h2 = self.serial_port.read(1)
            if h2[0] != RX_HEADER_2:
                continue
                
            cmd_bytes = self.serial_port.read(1)
            if not cmd_bytes:
                continue
            cmd = cmd_bytes[0]
            
            if cmd == CMD_ODOMETRY:
                # Payload: Vx, Vy, Wz (int16) -> 3 * 2 = 6 byte
                payload_len = 6 
                payload = self.serial_port.read(payload_len)
                
                if len(payload) == payload_len:
                    checksum_byte = self.serial_port.read(1)
                    if checksum_byte:
                        frame_to_check = bytes([RX_HEADER_1, RX_HEADER_2, cmd]) + payload
                        calc_chk = self.calculate_checksum(frame_to_check)
                        
                        if calc_chk == checksum_byte[0]:
                            self.publish_raw_velocity(payload)
                        else:
                            self.get_logger().warn("Checksum ERROR pada frame Odometri!")
            else:
                self.get_logger().warn(f"Command ID tidak dikenal: {cmd}")

    def publish_raw_velocity(self, payload):
        # Unpack 3 int16 Little-Endian
        vx_int, vy_int, wz_int = struct.unpack('<hhh', payload)
        
        # Konversi kembali ke float
        vx = vx_int / VEL_MULTIPLIER
        vy = vy_int / VEL_MULTIPLIER
        wz = wz_int / VEL_MULTIPLIER
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        
        self.raw_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
