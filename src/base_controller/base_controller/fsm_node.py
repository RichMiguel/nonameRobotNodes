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
            self.get_logger().warn("Robot sedang bergerak! Target baru ditolak atau antrikan.")
            # TODO: Logika pembatalan target lama (Preempt) bisa ditaruh di sini

    # --- Main FSM Loop ---
    def fsm_loop(self):
        if self.state == RobotState.CHECK:
            # 2. State: CHECK (Memastikan semua node dan sensor menyala)
            self.get_logger().info("CHECKING Kesiapan Sistem...", once=True)
            
            nav2_ready = self.nav_to_pose_client.wait_for_server(timeout_sec=0.1)
            
            if self.lidar_ready and self.odom_ready and nav2_ready:
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
            self.get_logger().info("Mengirim target ke Nav2 (Action Server)...")
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.current_goal
            
            # TODO: Gunakan send_goal_async() sesungguhnya untuk Nav2
            # Untuk sekarang kita asumsikan sukses dan langsung MOVE
            self.transition_to(RobotState.MOVE)
            
        elif self.state == RobotState.MOVE:
            # 5. State: MOVE
            # TODO: Pantau get_result_async() dari ActionClient
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
