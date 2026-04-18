#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import torch
import torch.nn as nn
import os
import math

# Replicating the DQN structure from train_rl.py natively
class DQNNetwork(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQNNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, 64)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, action_size)

    def forward(self, x):
        out = self.fc1(x)
        out = self.relu(out)
        out = self.fc2(out)
        out = self.relu(out)
        return self.fc3(out)

class DQNLocalPlannerNode(Node):
    def __init__(self):
        super().__init__('dqn_local_planner_node')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        
        # Layer 2 addition: Subscribe to Layer 1's path!
        self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.global_path = []
        
        self.state_size = 10 # (dx, dy, 8 lidar bins) matching rl_local_planner
        self.action_size = 4 # UP, DOWN, LEFT, RIGHT
        self.model = DQNNetwork(self.state_size, self.action_size)
        self.load_dqn_model()
        
        self.current_pose = None
        self.current_goal = None
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.last_scan = None
        self.get_logger().info("Layer 2: DQN Local Planner + Path Tracker Initialized.")

    def load_dqn_model(self):
        path = 'dqn_model.pth'
        if os.path.exists(path):
            try:
                self.model.load_state_dict(torch.load(path))
                self.model.eval()
            except Exception as e:
                self.get_logger().error(f"Failed to load PyTorch Weights: {e}")
        else:
            self.get_logger().warn("dqn_model.pth not found, running random/untrained weights.")

    def pose_callback(self, msg):
        self.current_pose = (msg.pose.position.x, msg.pose.position.y)

    def goal_callback(self, msg):
        # Fallback if no global path
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        
    def path_callback(self, msg):
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def scan_callback(self, msg):
        self.last_scan = msg

    def get_relative_state_tensor(self):
        if self.current_goal is None or self.last_scan is None or self.current_pose is None:
            return None
            
        dx_norm = (self.current_goal[0] - self.current_pose[0]) / 100.0 
        dy_norm = (self.current_goal[1] - self.current_pose[1]) / 100.0 
        
        ranges = self.last_scan.ranges
        num_rays = len(ranges)
        if num_rays == 0: return None
        
        bins = [0.0] * 8
        bin_size = num_rays // 8
        threshold = 2.0 
        
        for i in range(8):
            start = i * bin_size
            end = start + bin_size
            sector = ranges[start:end]
            if min(sector) < threshold:
                bins[i] = 1.0 # Boolean 1.0 for tensor
                
        # [dx, dy, up, down, left, right, up-left, up-right, dn-left, dn-right]
        state_list = [dx_norm, dy_norm, bins[0], bins[4], bins[2], bins[6], bins[1], bins[3], bins[5], bins[7]]
        return torch.FloatTensor(state_list).unsqueeze(0)

    def control_loop(self):
        # PATH TRACKING LOGIC (Carrot follow)
        if self.global_path and self.current_pose:
            min_dist = float('inf')
            closest_idx = 0
            for i, wp in enumerate(self.global_path):
                dist = math.hypot(wp[0] - self.current_pose[0], wp[1] - self.current_pose[1])
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            
            # Follow a point slightly ahead on the path
            carrot_idx = min(closest_idx + 3, len(self.global_path) - 1)
            self.current_goal = self.global_path[carrot_idx]
            
        state_tensor = self.get_relative_state_tensor()
        if state_tensor is None:
            return
            
        cmd = Twist()
        vel_scale = 1.0
        
        with torch.no_grad():
            q_values = self.model(state_tensor)
            action = torch.argmax(q_values).item()
            
        # Map Action: 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
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
    node = DQNLocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
