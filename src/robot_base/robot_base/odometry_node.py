#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Parameter Konfigurasi Odometri
        self.declare_parameter('odom_frequency', 50.0)      # Frekuensi publish TF & odom (50 Hz)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('vel_timeout', 0.5)          # Kecepatan dianggap 0 jika data serial terhenti > 0.5s
        self.declare_parameter('linear_scale_x', 1.1)       # Faktor kalibrasi skala translasi sumbu X
        self.declare_parameter('linear_scale_y', 1.0)       # Faktor kalibrasi skala translasi sumbu Y (strafe)
        self.declare_parameter('angular_scale_z', 0.9)      # Faktor kalibrasi skala rotasi Yaw
        
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.vel_timeout = self.get_parameter('vel_timeout').value
        self.scale_x = float(self.get_parameter('linear_scale_x').value)
        self.scale_y = float(self.get_parameter('linear_scale_y').value)
        self.scale_z = float(self.get_parameter('angular_scale_z').value)
        frequency = self.get_parameter('odom_frequency').value
        timer_period = 1.0 / frequency
        
        # Variabel State Odometri (Posisi Relatif Robot)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Cache Kecepatan Terkini dari Hardware
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        
        # Timer & Watchdog
        self.last_timer_time = self.get_clock().now()
        self.last_vel_msg_time = self.get_clock().now()
        
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
        
        # Pilar 2: Timer Periodik Mandiri (Continuous 50Hz TF Publisher)
        # Menjamin TF selalu ada dan tidak pernah starvation bahkan saat startup atau serial buffering
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"Odometry Node (Holonomic) aktif: publishing periodik pada {frequency} Hz.")

    def vel_callback(self, msg: TwistStamped):
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.wz = msg.twist.angular.z
        self.last_vel_msg_time = self.get_clock().now()

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_timer_time).nanoseconds / 1e9
        self.last_timer_time = current_time
        
        # Abaikan delta time abnormal (misal clock jump atau dt negatif)
        if dt <= 0.0 or dt > 1.0:
            return
            
        # Watchdog: jika data serial tidak masuk selama > vel_timeout, anggap robot diam
        vel_age = (current_time - self.last_vel_msg_time).nanoseconds / 1e9
        if vel_age > self.vel_timeout:
            active_vx = 0.0
            active_vy = 0.0
            active_wz = 0.0
        else:
            active_vx = self.vx * self.scale_x
            active_vy = self.vy * self.scale_y
            active_wz = self.wz * self.scale_z
            
        # Integrasi Kinematik untuk Robot Holonomic (2nd-Order Midpoint Integration)
        # Menggunakan sudut tengah (theta_mid) untuk mencegah akumulasi error saat robot berputar sambil bergerak
        delta_theta = active_wz * dt
        theta_mid = self.theta + (delta_theta * 0.5)
        
        delta_x = (active_vx * math.cos(theta_mid) - active_vy * math.sin(theta_mid)) * dt
        delta_y = (active_vx * math.sin(theta_mid) + active_vy * math.cos(theta_mid)) * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalisasi sudut yaw ke [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Konversi theta ke Quaternion
        q = euler_to_quaternion(0.0, 0.0, self.theta)
        msg_time = current_time.to_msg()
        
        # 1. Publish Transformasi (TF) odom -> base_link
        t = TransformStamped()
        t.header.stamp = msg_time
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
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
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Covariance Pose Standar
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[14] = 1000000.0
        odom.pose.covariance[21] = 1000000.0
        odom.pose.covariance[28] = 1000000.0
        odom.pose.covariance[35] = 0.001
        
        odom.twist.twist.linear.x = active_vx
        odom.twist.twist.linear.y = active_vy
        odom.twist.twist.angular.z = active_wz
        
        # Covariance Twist Standar
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[7] = 0.001
        odom.twist.covariance[14] = 1000000.0
        odom.twist.covariance[21] = 1000000.0
        odom.twist.covariance[28] = 1000000.0
        odom.twist.covariance[35] = 0.001
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
