#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32MultiArray
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformBroadcaster

SQRT2_2 = math.sqrt(2.0) / 2.0

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
        self.declare_parameter('linear_scale_x', 1.115)        # Faktor kalibrasi skala translasi sumbu X
        self.declare_parameter('linear_scale_y', 1.0)         # Faktor kalibrasi skala translasi sumbu Y (strafe)
        self.declare_parameter('angular_scale_z', 1.0)        # Faktor kalibrasi skala rotasi Yaw
        self.declare_parameter('wheel_radius', 0.045)         # Radius roda (meter) sesuai config.h
        self.declare_parameter('robot_radius', 0.24)          # Jarak pusat ke titik kontak roda (meter) sesuai config.h
        self.declare_parameter('encoder_ppr_total', 134.4)    # Total pulsa encoder per putaran output shaft (7.0 * 19.2)
        self.declare_parameter('max_tick_jump', 5000)         # Batas delta tick wajar per paket (anti glitch/reboot)
        
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.vel_timeout = self.get_parameter('vel_timeout').value
        self.scale_x = float(self.get_parameter('linear_scale_x').value)
        self.scale_y = float(self.get_parameter('linear_scale_y').value)
        self.scale_z = float(self.get_parameter('angular_scale_z').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.ppr_total = float(self.get_parameter('encoder_ppr_total').value)
        self.max_tick_jump = int(self.get_parameter('max_tick_jump').value)
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ppr_total
        frequency = self.get_parameter('odom_frequency').value
        timer_period = 1.0 / frequency
        
        # Variabel State Odometri (Posisi Relatif Robot)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # State Encoder Ticks Kumulatif (Primary Industrial Odometry)
        self.using_ticks = False
        self.prev_ticks = None
        self.last_ticks_time = self.get_clock().now()
        
        # Cache Kecepatan Terkini (untuk Twist & Fallback jika firmware lama)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        
        # Timer & Watchdog
        self.last_timer_time = self.get_clock().now()
        self.last_vel_msg_time = self.get_clock().now()
        
        # Subscribe ke raw cumulative encoder ticks (utama) & raw_odom_vel (fallback)
        self.ticks_sub = self.create_subscription(
            Int32MultiArray,
            'wheel_ticks',
            self.ticks_callback,
            10
        )
        self.vel_sub = self.create_subscription(
            TwistStamped,
            'raw_odom_vel',
            self.vel_callback,
            10
        )
        
        # Publisher standar Nav2
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber untuk Reset Odometri
        self.reset_odom_sub = self.create_subscription(
            Empty,
            'reset_odom',
            self.reset_odom_callback,
            10
        )

        # Dynamic Parameter Updates Callback (Kalibrasi Skala Runtime)
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Pilar 2: Timer Periodik Mandiri (Continuous 50Hz TF Publisher)
        # Menjamin TF selalu ada dan tidak pernah starvation bahkan saat startup atau serial buffering
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"Odometry Node (Holonomic) aktif: publishing periodik pada {frequency} Hz.")

    def reset_odom_callback(self, msg: Empty):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info("✅ Odometri berhasil di-RESET ke (0.0, 0.0, 0.0)")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'linear_scale_x':
                self.scale_x = float(param.value)
                self.get_logger().info(f"🔄 Parameter linear_scale_x diperbarui: {self.scale_x}")
            elif param.name == 'linear_scale_y':
                self.scale_y = float(param.value)
                self.get_logger().info(f"🔄 Parameter linear_scale_y diperbarui: {self.scale_y}")
            elif param.name == 'angular_scale_z':
                self.scale_z = float(param.value)
                self.get_logger().info(f"🔄 Parameter angular_scale_z diperbarui: {self.scale_z}")
        return SetParametersResult(successful=True)

    def ticks_callback(self, msg: Int32MultiArray):
        """Menghitung perpindahan eksak robot langsung dari delta tick encoder kumulatif."""
        if len(msg.data) < 4:
            return

        now = self.get_clock().now()
        ticks = list(msg.data[:4])

        if not self.using_ticks:
            self.using_ticks = True
            self.get_logger().info("✅ Mode Odometri Industri aktif: menggunakan akumulasi tick encoder (wheel_ticks).")

        if self.prev_ticks is None:
            self.prev_ticks = ticks
            self.last_ticks_time = now
            return

        delta_ticks = [t - p for t, p in zip(ticks, self.prev_ticks)]

        # Guard terhadap reset Arduino atau lonjakan glitch serial
        if any(abs(d) > self.max_tick_jump for d in delta_ticks):
            self.get_logger().warn(
                f"⚠️ Lonjakan/reset encoder terdeteksi ({delta_ticks}), melakukan re-sync baseline tick."
            )
            self.prev_ticks = ticks
            self.last_ticks_time = now
            return

        dt_ticks = (now - self.last_ticks_time).nanoseconds / 1e9
        self.prev_ticks = ticks
        self.last_ticks_time = now

        # Konversi delta tick tiap roda menjadi perpindahan linier roda (meter)
        ds1, ds2, ds3, ds4 = (d * self.meters_per_tick for d in delta_ticks)

        # Forward Kinematics X-Drive (REP-103, selaras dengan XDriveKinematics.cpp)
        denom = 4.0 * SQRT2_2
        dx_local = ((ds1 - ds2 + ds3 - ds4) / denom) * self.scale_x
        dy_local = ((-ds1 - ds2 + ds3 + ds4) / denom) * self.scale_y
        d_theta = (-(ds1 + ds2 + ds3 + ds4) / (4.0 * self.robot_radius)) * self.scale_z

        # 2nd-Order Runge-Kutta (Midpoint Integration) dari perpindahan eksak
        theta_mid = self.theta + (d_theta * 0.5)
        self.x += dx_local * math.cos(theta_mid) - dy_local * math.sin(theta_mid)
        self.y += dx_local * math.sin(theta_mid) + dy_local * math.cos(theta_mid)
        self.theta = math.atan2(math.sin(self.theta + d_theta), math.cos(self.theta + d_theta))

        # Hitung twist aktual untuk pesan Odometry (tidak memengaruhi integrasi posisi)
        if dt_ticks > 0.001:
            self.vx = dx_local / dt_ticks
            self.vy = dy_local / dt_ticks
            self.wz = d_theta / dt_ticks
        self.last_vel_msg_time = now

    def vel_callback(self, msg: TwistStamped):
        if self.using_ticks:
            return
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
            
        # Watchdog: jika data serial tidak masuk selama > vel_timeout, anggap kecepatan 0
        vel_age = (current_time - self.last_vel_msg_time).nanoseconds / 1e9
        if vel_age > self.vel_timeout:
            active_vx = 0.0
            active_vy = 0.0
            active_wz = 0.0
        elif self.using_ticks:
            # Posisi sudah terintegrasi eksak di ticks_callback; twist langsung pakai hasil delta/dt
            active_vx = self.vx
            active_vy = self.vy
            active_wz = self.wz
        else:
            # Fallback mode kecepatan (jika firmware belum mengirim CMD_ODOM_TICKS)
            active_vx = self.vx * self.scale_x
            active_vy = self.vy * self.scale_y
            active_wz = self.wz * self.scale_z

            delta_theta = active_wz * dt
            theta_mid = self.theta + (delta_theta * 0.5)
            delta_x = (active_vx * math.cos(theta_mid) - active_vy * math.sin(theta_mid)) * dt
            delta_y = (active_vx * math.sin(theta_mid) + active_vy * math.cos(theta_mid)) * dt

            self.x += delta_x
            self.y += delta_y
            self.theta = math.atan2(math.sin(self.theta + delta_theta), math.cos(self.theta + delta_theta))
        
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
