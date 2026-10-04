#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Variabel State Odometri (Posisi Relatif Robot)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Menyimpan waktu terakhir pesan diterima untuk menghitung dt
        self.last_time = self.get_clock().now()
        
        # Subscribe ke kecepatan mentah (Vx, Vy, Wz) dari communication_node
        self.vel_sub = self.create_subscription(
            TwistStamped,
            'raw_odom_vel',
            self.vel_callback,
            10
        )
        
        # Publisher standar Nav2
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info("Odometry Node (Holonomic) telah berjalan.")

    def vel_callback(self, msg: TwistStamped):
        current_time = self.get_clock().now()
        
        # Hitung selisih waktu (delta time) dalam detik
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Dapatkan kecepatan di koordinat lokal robot (base_link)
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        wz = msg.twist.angular.z
        
        # Integrasi Kinematik untuk Robot Holonomic
        # Mengubah pergerakan lokal (base_link) menjadi pergerakan global (odom)
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = wz * dt
        
        # Update posisi terkini
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Konversi theta (Yaw) ke Quaternion untuk ROS 2
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        
        msg_time = current_time.to_msg()
        
        # 1. Publish Transformasi (TF) dari odom -> base_link
        t = TransformStamped()
        t.header.stamp = msg_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Publish topik /odom
        odom = Odometry()
        odom.header.stamp = msg_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Data Posisi (Pose)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Data Kecepatan (Twist) - Kecepatan pada frame base_link
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        
        # Optional: Tambahkan covariance matrix di sini jika Anda menggunakan filter seperti EKF (robot_localization)
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
