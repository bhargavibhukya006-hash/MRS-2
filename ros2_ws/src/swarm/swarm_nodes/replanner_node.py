#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import math
import time

class ReplannerNode(Node):
    def __init__(self):
        super().__init__('replanner_node')
        
        # Subscriptions to monitor system health
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        # Publisher to trigger Layer 1 (A*) Recalculation
        self.dynamic_obs_pub = self.create_publisher(Point, 'add_dynamic_obstacle', 10)
        
        self.current_pose = None
        self.current_path = []
        self.last_scan = None
        
        # Obstacle Persistence Tracking (Wait for sometime logic)
        self.path_obstructed_start_time = None
        self.OBSTACLE_PATIENCE_SECONDS = 3.0 # "Wait for sometime if obstacle is not moving"
        
        # Stuck Tracking logic (No progress tracking)
        self.pos_history = []
        
        # Master evaluation loop runs at 5Hz
        self.create_timer(0.2, self.evaluation_loop)
        
        self.get_logger().info("Layer 3: Replanner Supervisor Active. Monitoring Path Health.")

    def pose_callback(self, msg):
        self.current_pose = (msg.pose.position.x, msg.pose.position.y)

    def path_callback(self, msg):
        # A new path was requested, reset all triggers
        self.current_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_obstructed_start_time = None
        self.pos_history.clear()

    def scan_callback(self, msg):
        self.last_scan = msg

    def evaluation_loop(self):
        if not self.current_pose or not self.current_path or not self.last_scan:
            return

        # ==========================================
        # CONDITION 1: IS THE ROBOT STUCK LOCALLY?
        # ==========================================
        self.pos_history.append((self.current_pose[0], self.current_pose[1], time.time()))
        
        # Prune old history > 5 seconds ago
        current_time = time.time()
        self.pos_history = [p for p in self.pos_history if current_time - p[2] < 5.0]
        
        if len(self.pos_history) > 10: # Enough data collected
            oldest = self.pos_history[0]
            dist_moved = math.hypot(self.current_pose[0] - oldest[0], self.current_pose[1] - oldest[1])
            if dist_moved < 1.0: # Moved less than 1 meter in 5 seconds
                self.get_logger().error("REPLAN TRIGGER: Robot is physically STUCK. Triggering A* Recalculate!")
                self.request_replan_at(self.current_pose[0], self.current_pose[1])
                return

        # ==========================================
        # CONDITION 2: Lidar detects obstacle blocking path ahead
        # ==========================================
        obstacle_on_path = False
        detected_obs_x = 0.0
        detected_obs_y = 0.0
        
        for i in range(len(self.last_scan.ranges)):
            r = self.last_scan.ranges[i]
            if r < self.last_scan.range_max:
                angle = self.last_scan.angle_min + i * self.last_scan.angle_increment
                obs_x = self.current_pose[0] + r * math.cos(angle)
                obs_y = self.current_pose[1] + r * math.sin(angle)
                
                # Check if this lidar hit touches our A* route within a 15m lookahead
                for (px, py) in self.current_path:
                    dist_to_path_node = math.hypot(px - self.current_pose[0], py - self.current_pose[1])
                    if dist_to_path_node < 15.0:
                        obs_dist = math.hypot(px - obs_x, py - obs_y)
                        if obs_dist < 4.0: # Object is 4m away from our formal global route
                            obstacle_on_path = True
                            detected_obs_x = obs_x
                            detected_obs_y = obs_y
                            break
                if obstacle_on_path:
                    break

        # ==========================================
        # WAITING LOGIC: "WAIT FOR SOMETIME IF OBSTACLE IS NOT MOVING"
        # ==========================================
        if obstacle_on_path:
            if self.path_obstructed_start_time is None:
                # Obstacle just appeared! Start the timer.
                self.get_logger().info("Obstacle detected on global path. Waiting to see if it moves...")
                self.path_obstructed_start_time = current_time
            else:
                elapsed = current_time - self.path_obstructed_start_time
                if elapsed > self.OBSTACLE_PATIENCE_SECONDS:
                    # Time's up! Obstacle is still blocking the way.
                    self.get_logger().warn(f"REPLAN TRIGGER: Obstacle blocked path for > {self.OBSTACLE_PATIENCE_SECONDS}s. Forcing A* Reroute!")
                    # Corner Case Handler: Only submit the obstacle slightly ahead of us so it forces a detour
                    self.request_replan_at(detected_obs_x, detected_obs_y)
        else:
            if self.path_obstructed_start_time is not None:
                self.get_logger().info("Obstacle cleared! Path is safe again. No replan needed.")
                self.path_obstructed_start_time = None

    def request_replan_at(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.dynamic_obs_pub.publish(msg)
        # Clear triggers so we don't spam while A* computes
        self.path_obstructed_start_time = None
        self.pos_history.clear()

def main(args=None):
    rclpy.init(args=args)
    node = ReplannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
