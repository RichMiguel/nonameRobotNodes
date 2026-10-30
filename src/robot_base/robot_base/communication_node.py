#!/usr/bin/env python3
"""
Node Komunikasi Serial ROS 2 <-> Arduino Mega 2560 (Low-Level Firmware)
Menerima /cmd_vel dari Nav2/teleop, mengemas ke dalam frame biner deterministik,
dan membaca umpan balik odometri kecepatan robot (actual vel) dari firmware
menggunakan Finite State Machine (FSM) buffer stream parser tahan gangguan.
"""

import rclpy
from rclpy.node import Node
import serial
import struct
import time
from geometry_msgs.msg import Twist, TwistStamped

# Konstanta Protokol (ROS2 -> Arduino)
TX_HEADER_1 = 0xAA
TX_HEADER_2 = 0x55
CMD_SET_VELOCITY = 0x01

# Konstanta Protokol (Arduino -> ROS2)
RX_HEADER_1 = 0xBB
RX_HEADER_2 = 0x66
CMD_ODOMETRY = 0x01

# Faktor Pengali (Multiplier) untuk konversi Float (m/s & rad/s) ke Int16 (mm/s & mrad/s)
VEL_MULTIPLIER = 1000.0

# Panjang Paket Odometri: Header (2) + CMD (1) + Payload (6) + Checksum (1) = 10 byte
FEEDBACK_FRAME_LEN = 10


class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('reconnect_interval', 1.0)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        
        self.serial_port = None
        self.last_reconnect_attempt = 0.0
        self._rx_buffer = bytearray()
        
        # Buka koneksi awal
        self.try_connect_serial()

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.raw_vel_pub = self.create_publisher(
            TwistStamped,
            'raw_odom_vel',
            10
        )
        
        # Loop periodik pembacaan serial & watchdog koneksi (50 Hz / 20 ms)
        self.serial_timer = self.create_timer(0.02, self.serial_io_loop)
        
        self.get_logger().info(
            f"Communication Node aktif di port '{self.port}' @ {self.baudrate} baud."
        )

    def try_connect_serial(self):
        """Membuka atau menyambung ulang port serial dengan penanganan error yang aman."""
        if self.serial_port is not None and self.serial_port.is_open:
            return True

        now = time.time()
        if (now - self.last_reconnect_attempt) < self.reconnect_interval:
            return False

        self.last_reconnect_attempt = now
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0,       # Non-blocking read murni
                write_timeout=0.05
            )
            self._rx_buffer.clear()
            self.get_logger().info(f"✅ Berhasil terkoneksi ke low-level di {self.port}")
            return True
        except serial.SerialException as e:
            self.get_logger().warn(
                f"Koneksi ke serial {self.port} gagal ({e}). Akan mencoba lagi...",
                throttle_duration_sec=5.0
            )
            self.serial_port = None
            return False

    def close_serial(self):
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        checksum = 0
        for b in data:
            checksum ^= b
        return checksum

    def cmd_vel_callback(self, msg: Twist):
        """Mengemas Twist linear (vx, vy) dan angular (wz) ke frame biner 10 byte."""
        if self.serial_port is None or not self.serial_port.is_open:
            return
            
        vx = int(msg.linear.x * VEL_MULTIPLIER)
        vy = int(msg.linear.y * VEL_MULTIPLIER)
        wz = int(msg.angular.z * VEL_MULTIPLIER)
        
        # Constrain ke range int16 [-32768, 32767]
        vx = max(min(vx, 32767), -32768)
        vy = max(min(vy, 32767), -32768)
        wz = max(min(wz, 32767), -32768)
        
        # Little-Endian format: 3 x int16
        payload = struct.pack('<hhh', vx, vy, wz)
        
        # Checksum = XOR (Command ID + Payload)
        data_to_checksum = bytes([CMD_SET_VELOCITY]) + payload
        checksum = self.calculate_checksum(data_to_checksum)
        
        frame = bytes([TX_HEADER_1, TX_HEADER_2, CMD_SET_VELOCITY]) + payload + bytes([checksum])
        
        try:
            self.serial_port.write(frame)
        except serial.SerialException as e:
            self.get_logger().error(f"Error transmisi serial: {e}")
            self.close_serial()

    def serial_io_loop(self):
        """Loop non-blocking untuk membaca stream serial dengan Finite State Machine buffer parser."""
        if self.serial_port is None or not self.serial_port.is_open:
            self.try_connect_serial()
            return
            
        try:
            bytes_waiting = self.serial_port.in_waiting
            if bytes_waiting > 0:
                raw_chunk = self.serial_port.read(bytes_waiting)
                if raw_chunk:
                    self._rx_buffer.extend(raw_chunk)
        except serial.SerialException as e:
            self.get_logger().error(f"Error membaca serial port: {e}")
            self.close_serial()
            return

        # FSM Buffer Stream Parsing (Tahan banting terhadap packet drop & noise)
        while len(self._rx_buffer) >= FEEDBACK_FRAME_LEN:
            # 1. Cari Header 1 (0xBB)
            if self._rx_buffer[0] != RX_HEADER_1:
                del self._rx_buffer[0]
                continue

            # 2. Periksa Header 2 (0x66)
            if self._rx_buffer[1] != RX_HEADER_2:
                del self._rx_buffer[0]
                continue

            # Buffer memiliki setidaknya 1 frame lengkap
            cmd = self._rx_buffer[2]
            payload = bytes(self._rx_buffer[3:9])
            received_checksum = self._rx_buffer[9]

            # Verifikasi Checksum XOR
            calc_checksum = self.calculate_checksum(bytes([cmd]) + payload)
            if calc_checksum == received_checksum:
                # Checksum Valid -> Ekstraksi dan publish
                if cmd == CMD_ODOMETRY:
                    self.publish_raw_velocity(payload)
                # Geser buffer melompati 1 frame valid
                del self._rx_buffer[:FEEDBACK_FRAME_LEN]
            else:
                # Checksum tidak cocok -> Geser 1 byte untuk mencari sinkronisasi frame berikutnya
                self.get_logger().warn(
                    f"Checksum mismatch pada frame biner (calc: {calc_checksum}, got: {received_checksum})",
                    throttle_duration_sec=2.0
                )
                del self._rx_buffer[0]

        # Cegah akumulasi buffer liar jika tidak ada header yang cocok
        if len(self._rx_buffer) > 256:
            self._rx_buffer.clear()

    def publish_raw_velocity(self, payload: bytes):
        """Membongkar payload 6-byte int16 ke TwistStamped dengan timestamp waktu terima."""
        vx_int, vy_int, wz_int = struct.unpack('<hhh', payload)
        
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

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


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
