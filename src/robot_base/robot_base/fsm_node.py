#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
import time

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan

class RobotState(Enum):
    BOOT = 1
    CHECK = 2
    LISTEN = 3
    CALCULATE = 4
    MOVE = 5
    RECALCULATE = 6
    STOP = 7

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')
        
        self.declare_parameter('bypass_health_check', False)
        
        # 1. State: BOOT
        self.state = RobotState.BOOT
        self.get_logger().info("System BOOTING: Inisialisasi FSM Node...")
        
        # Bendera Kesiapan (Health Flags)
        self.lidar_ready = False
        self.odom_ready = False
        self.current_goal = None
        
        # Subscribers untuk health check
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(TwistStamped, 'raw_odom_vel', self.odom_callback, 10)
        
        # Subscriber untuk menerima target tujuan dari RViz2/Dashboard
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        
        # Action Client ke Nav2 Behavior Tree Navigator
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Loop utama FSM (berjalan setiap 0.5 detik)
        self.fsm_timer = self.create_timer(0.5, self.fsm_loop)
        
        # Selesai BOOT, langsung pindah ke CHECK
        self.transition_to(RobotState.CHECK)

    def transition_to(self, new_state):
        self.get_logger().info(f"[TRANSITION] {self.state.name} -> {new_state.name}")
        self.state = new_state

    # --- Callbacks ---
    def scan_callback(self, msg):
        self.lidar_ready = True

    def odom_callback(self, msg):
        self.odom_ready = True

    def goal_callback(self, msg: PoseStamped):
        if self.state == RobotState.LISTEN:
            self.get_logger().info("Target navigasi baru diterima!")
            self.current_goal = msg
            self.transition_to(RobotState.CALCULATE)
        elif self.state == RobotState.MOVE:
            self.get_logger().warn("Robot sedang bergerak! Target baru menggantikan target lama (Preempting).")
            self.current_goal = msg
            # Kita pindah ke calculate untuk mengirim target baru
            self.transition_to(RobotState.CALCULATE)

    # --- Callbacks untuk Action Client Nav2 ---
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Target ditolak oleh Nav2! (Mungkin di luar jangkauan atau error waktu)")
                self.transition_to(RobotState.LISTEN)
                return
                
            self.get_logger().info("Target diterima oleh Nav2. Robot mulai bergerak...")
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error saat menerima respon goal: {e}")
            self.transition_to(RobotState.LISTEN)

    def get_result_callback(self, future):
        # Saat hasil dari Nav2 kembali, ubah state kembali ke LISTEN
        # pastikan tidak sedang di-preempt oleh CALCULATE yang baru
        if self.state == RobotState.MOVE:
            try:
                status = future.result().status
                if status == 4: # 4 = SUCCEEDED
                    self.get_logger().info("Robot telah SAMPAI di tujuan!")
                else:
                    self.get_logger().warn(f"Navigasi dibatalkan atau gagal. (Status code: {status})")
            except Exception as e:
                self.get_logger().error(f"Error saat mendapat hasil navigasi: {e}")
            
            self.transition_to(RobotState.LISTEN)

    # --- Main FSM Loop ---
    def fsm_loop(self):
        if self.state == RobotState.CHECK:
            # 2. State: CHECK (Memastikan semua node dan sensor menyala)
            self.get_logger().info("CHECKING Kesiapan Sistem...", once=True)
            
            bypass = self.get_parameter('bypass_health_check').get_parameter_value().bool_value
            nav2_ready = self.nav_to_pose_client.wait_for_server(timeout_sec=0.1)
            
            if bypass or (self.lidar_ready and self.odom_ready and nav2_ready):
                if bypass:
                    self.get_logger().warn("BYPASS MODE AKTIF: Mengabaikan sensor hardware. Memulai LISTEN.")
                else:
                    self.get_logger().info("Semua sistem SIAP (Lidar, Odom, Nav2). Memulai LISTEN.")
                self.transition_to(RobotState.LISTEN)
            else:
                self.get_logger().debug(f"Menunggu... Lidar:{self.lidar_ready}, Odom:{self.odom_ready}, Nav2:{nav2_ready}")
                
        elif self.state == RobotState.LISTEN:
            # 3. State: LISTEN
            # Diam menunggu goal_callback dipanggil
            pass
            
        elif self.state == RobotState.CALCULATE:
            # 4. State: CALCULATE
            self.get_logger().info("Mengirim target posisi ke Nav2 Action Server...")
            
            # Update timestamp ke waktu simulasi terkini agar tidak ditolak oleh Nav2
            self.current_goal.header.stamp = self.get_clock().now().to_msg()
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.current_goal
            
            # Kirim goal ke Nav2 secara Asynchronous
            self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            
            self.transition_to(RobotState.MOVE)
            
        elif self.state == RobotState.MOVE:
            # 5. State: MOVE
            # Diam menunggu respon dari ActionClient (melalui get_result_callback)
            pass
            
        elif self.state == RobotState.RECALCULATE:
            # 6. State: RECALCULATE
            pass
            
        elif self.state == RobotState.STOP:
            # 7. State: STOP
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
