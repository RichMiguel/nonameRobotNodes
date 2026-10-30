#!/usr/bin/env python3
"""
Node Kinematika Invers Robot Holonomik X-Drive.
Menghitung kecepatan sudut 4 roda omnidirectional dari perintah /cmd_vel (vx, vy, wz)
dan mengirimkan perintah kecepatan ke Gazebo JointController serta mem-publish /joint_states.

Konfigurasi Geometri (X-Drive 45 Derajat):
- Front Left (FL)  : Posisi (+Lx, +Ly), Orientasi yaw 135°
- Front Right (FR) : Posisi (+Lx, -Ly), Orientasi yaw 45°
- Rear Left (RL)   : Posisi (-Lx, +Ly), Orientasi yaw -135°
- Rear Right (RR)  : Posisi (-Lx, -Ly), Orientasi yaw -45°
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

class XDriveKinematicsNode(Node):
    def __init__(self):
        super().__init__('xdrive_kinematics_node')

        # Parameter Geometri Robot (Sinkron dengan Firmware: Roda 45mm, Radius Robot 24cm)
        self.declare_parameter('wheel_radius', 0.045)       # Jari-jari roda (meter) = 45 mm
        self.declare_parameter('wheel_offset_x', 0.1697)    # Jarak x pusat ke roda (meter) -> sqrt(x^2 + y^2) = 0.24m
        self.declare_parameter('wheel_offset_y', 0.1697)    # Jarak y pusat ke roda (meter)
        self.declare_parameter('cmd_timeout', 0.5)          # Timeout berhenti jika tidak ada cmd_vel (detik)
        self.declare_parameter('publish_rate', 50.0)        # Frekuensi update loop (Hz)

        self.R = float(self.get_parameter('wheel_radius').value)
        self.Lx = float(self.get_parameter('wheel_offset_x').value)
        self.Ly = float(self.get_parameter('wheel_offset_y').value)
        self.timeout = float(self.get_parameter('cmd_timeout').value)
        rate = float(self.get_parameter('publish_rate').value)

        # Konstanta Kinematika
        # c = sqrt(2)/2 = cos(45°) = sin(45°) ≈ 0.70710678
        self.c = math.sqrt(2) / 2.0
        # L = Lx + Ly = Jarak efektif rotasi
        self.L = self.Lx + self.Ly

        # State kecepatan dan posisi sudut roda (rad & rad/s)
        self.w_fl = 0.0
        self.w_fr = 0.0
        self.w_rl = 0.0
        self.w_rr = 0.0

        self.pos_fl = 0.0
        self.pos_fr = 0.0
        self.pos_rl = 0.0
        self.pos_rr = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_loop_time = self.get_clock().now()

        # Subscriber /cmd_vel
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher kecepatan roda ke Gazebo (menyesuaikan topic default gz-sim-joint-controller)
        self.pub_fl = self.create_publisher(Float64, '/model/xdrive/joint/front_left_wheel_joint/cmd_vel', 10)
        self.pub_fr = self.create_publisher(Float64, '/model/xdrive/joint/front_right_wheel_joint/cmd_vel', 10)
        self.pub_rl = self.create_publisher(Float64, '/model/xdrive/joint/rear_left_wheel_joint/cmd_vel', 10)
        self.pub_rr = self.create_publisher(Float64, '/model/xdrive/joint/rear_right_wheel_joint/cmd_vel', 10)

        # Publisher diagnostik (MultiArray)
        self.pub_wheel_speeds = self.create_publisher(Float64MultiArray, '/wheel_velocities', 10)

        # Publisher /joint_states agar visualisasi roda di RViz/TF selalu berputar sinkron
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)

        # Timer loop periodik (50 Hz)
        self.timer = self.create_timer(1.0 / rate, self.timer_loop)

        self.get_logger().info(
            f"X-Drive Kinematics Node aktif. Roda: R={self.R}m, Lx={self.Lx}m, Ly={self.Ly}m"
        )

    def compute_inverse_kinematics(self, vx: float, vy: float, wz: float):
        """
        Menghitung kecepatan sudut 4 roda (rad/s) dari kecepatan robot (vx, vy, wz):
        v_fl = -c * vx + c * vy + (c * L) * wz
        v_fr =  c * vx + c * vy + (c * L) * wz
        v_rl = -c * vx - c * vy + (c * L) * wz
        v_rr =  c * vx - c * vy + (c * L) * wz
        omega = v / R
        """
        cL = self.c * self.L

        v_fl = -self.c * vx + self.c * vy + cL * wz
        v_fr =  self.c * vx + self.c * vy + cL * wz
        v_rl = -self.c * vx - self.c * vy + cL * wz
        v_rr =  self.c * vx - self.c * vy + cL * wz

        w_fl = v_fl / self.R
        w_fr = v_fr / self.R
        w_rl = v_rl / self.R
        w_rr = v_rr / self.R

        return w_fl, w_fr, w_rl, w_rr

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        self.w_fl, self.w_fr, self.w_rl, self.w_rr = self.compute_inverse_kinematics(vx, vy, wz)

    def timer_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now

        # Watchdog timeout: jika tidak ada perintah cmd_vel baru, hentikan roda
        time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.timeout:
            self.w_fl = 0.0
            self.w_fr = 0.0
            self.w_rl = 0.0
            self.w_rr = 0.0

        # Integrasi posisi sudut roda untuk JointState
        if dt > 0.0:
            self.pos_fl += self.w_fl * dt
            self.pos_fr += self.w_fr * dt
            self.pos_rl += self.w_rl * dt
            self.pos_rr += self.w_rr * dt

        # 1. Publish perintah kecepatan masing-masing joint ke Gazebo
        msg_fl = Float64(data=self.w_fl)
        msg_fr = Float64(data=self.w_fr)
        msg_rl = Float64(data=self.w_rl)
        msg_rr = Float64(data=self.w_rr)

        self.pub_fl.publish(msg_fl)
        self.pub_fr.publish(msg_fr)
        self.pub_rl.publish(msg_rl)
        self.pub_rr.publish(msg_rr)

        # 2. Publish topik diagnostik /wheel_velocities
        diag_msg = Float64MultiArray()
        diag_msg.data = [self.w_fl, self.w_fr, self.w_rl, self.w_rr]
        self.pub_wheel_speeds.publish(diag_msg)

        # 3. Publish /joint_states
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        js.position = [self.pos_fl, self.pos_fr, self.pos_rl, self.pos_rr]
        js.velocity = [self.w_fl, self.w_fr, self.w_rl, self.w_rr]
        self.pub_joint_states.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = XDriveKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
