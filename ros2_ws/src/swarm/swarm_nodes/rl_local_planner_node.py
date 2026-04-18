#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import pickle
import os

class RLLocalPlannerNode(Node):
    def __init__(self):
        super().__init__('rl_local_planner_node')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        
        self.q_table = {}
        self.load_q_table()
        
        self.current_pose = (0.0, 0.0)
        self.current_goal = None
        self.grid_size = 10.0 # From original python
        
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz
        
        self.last_scan = None
        self.get_logger().info("RL Local Planner Initialized.")

    def load_q_table(self):
        try:
            # Default to current directory instead of hardcoded MRS-2 path
            path = 'q_table.pkl'
            if os.path.exists(path):
                with open(path, 'rb') as f:
                    self.q_table = pickle.load(f)
                self.get_logger().info(f"Loaded Q-table with {len(self.q_table)} states.")
            else:
                self.get_logger().warn("q_table.pkl not found, running randomly.")
        except Exception as e:
            self.get_logger().error(f"Could not load Q-table: {e}")

    def pose_callback(self, msg):
        self.current_pose = (msg.pose.position.x, msg.pose.position.y)

    def goal_callback(self, msg):
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)

    def scan_callback(self, msg):
        self.last_scan = msg

    def get_relative_state(self):
        if self.current_goal is None or self.last_scan is None:
            return None
            
        dx_raw = self.current_goal[0] - self.current_pose[0]
        dy_raw = self.current_goal[1] - self.current_pose[1]
        
        # Norm distances (similar to training grid scale)
        # Magic scaling factor based on max warehouse size
        dx_norm = dx_raw / 100.0 
        dy_norm = dy_raw / 100.0 
        
        # Process continuous Lidar (LaserScan) into discrete 8-directional bins
        ranges = self.last_scan.ranges
        num_rays = len(ranges)
        if num_rays == 0: return None
        
        # Bin ranges into 8 directions (w_up, w_down, etc. replacing grid blocks)
        # For simplicity, assuming a 360 degree scanner with 0 forward
        bins = [False] * 8
        bin_size = num_rays // 8
        threshold = 2.0 # 2 meters obstacle detection threshold
        
        for i in range(8):
            start = i * bin_size
            end = start + bin_size
            sector = ranges[start:end]
            if min(sector) < threshold:
                bins[i] = True
                
        # Up, down, left, right... matching the 8 bools of RL train phase
        return (dx_norm, dy_norm, bins[0], bins[4], bins[2], bins[6], bins[1], bins[3], bins[5], bins[7])

    def control_loop(self):
        state = self.get_relative_state()
        if state is None:
            return
            
        cmd = Twist()
        vel_scale = 1.0
        
        if state in self.q_table:
            action = max(self.q_table[state], key=self.q_table[state].get)
        else:
            import random
            action = random.choice([0,1,2,3]) # Explore fallback
            
        # Map Action: 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
        # To ROS Twist:
        if action == 0:
            cmd.linear.x = vel_scale
        elif action == 1:
            cmd.linear.x = -vel_scale
        elif action == 2:
            cmd.linear.y = -vel_scale
        elif action == 3:
            cmd.linear.y = vel_scale
            
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RLLocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
