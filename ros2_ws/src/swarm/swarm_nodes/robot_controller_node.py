#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import math
import random
import math
import random
import time

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.robot_id = self.get_name()
        
        # Continuous state
        self.x = random.uniform(100.0, 800.0)
        self.y = random.uniform(100.0, 600.0)
        
        self.target_vel_x = 0.0
        self.target_vel_y = 0.0
        
        self.battery = 100.0
        
        self.pos_history = []
        self.is_stuck = False
        self.ticks = 0
        
        # Random Dynamic Obstacles state
        # Each obs is a dict: {'x': x, 'y': y, 'vx': vx, 'vy': vy, 'expires': timestamp}
        self.dynamic_obstacles = []
        
        # Publishers & Subscribers
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.heartbeat_pub = self.create_publisher(String, 'heartbeat', 10)
        
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.nav_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_nav_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Timers
        self.create_timer(1.0, self.heartbeat_timer)
        self.create_timer(0.1, self.physics_loop)  # 10Hz physics/movement
        self.create_timer(0.5, self.manage_dynamic_obstacles) # Dynamic Obastacle Spawner
        
        self.current_goal = None
        self.get_logger().info(f"{self.robot_id} Initialized at ({self.x:.1f}, {self.y:.1f})")

    def manage_dynamic_obstacles(self):
        current_time = time.time()
        
        # 1. Age and Evolve existing obstacles
        alive_obstacles = []
        for obs in self.dynamic_obstacles:
            # Move them slightly
            obs['x'] += obs['vx'] * 0.5
            obs['y'] += obs['vy'] * 0.5
            # Bounce off walls gently bounds 0-1000
            if obs['x'] < 50 or obs['x'] > 950: obs['vx'] *= -1
            if obs['y'] < 50 or obs['y'] > 950: obs['vy'] *= -1
            
            # Keep if not expired
            if current_time < obs['expires']:
                alive_obstacles.append(obs)
                
        self.dynamic_obstacles = alive_obstacles
        
        # 2. Spawn new random obstacles if we have less than 5
        spawn_rate = 0.2 # 20% chance to spawn a new one every tick if room exists
        if len(self.dynamic_obstacles) < 7 and random.random() < spawn_rate:
            # Spawn roughly near the robot so they actually matter, but not exactly ON the robot
            spawn_x = self.x + random.uniform(-100, 100)
            spawn_y = self.y + random.uniform(-100, 100)
            
            new_obs = {
                'x': max(50.0, min(950.0, spawn_x)),
                'y': max(50.0, min(950.0, spawn_y)),
                'vx': random.uniform(-10.0, 10.0), # drift speed
                'vy': random.uniform(-10.0, 10.0),
                'expires': current_time + random.uniform(5.0, 25.0) # Vanish in 5-25 seconds
            }
            self.dynamic_obstacles.append(new_obs)

    def heartbeat_timer(self):
        msg = String()
        if self.is_stuck:
            msg.data = f"STATUS_BLOCKED_{self.battery:.1f}"
        else:
            msg.data = f"OK_BATTERY_{self.battery:.1f}"
        self.heartbeat_pub.publish(msg)

    def cmd_vel_callback(self, msg):
        self.target_vel_x = msg.linear.x * 150.0 
        self.target_vel_y = msg.linear.y * 150.0

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: {goal_request.pose.pose.position.x}, {goal_request.pose.pose.position.y}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_nav_callback(self, goal_handle):
        self.get_logger().info('Executing navigation...')
        target_x = goal_handle.request.pose.pose.position.x
        target_y = goal_handle.request.pose.pose.position.y
        
        self.current_goal = (target_x, target_y)
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled. Stopping in place!')
                self.current_goal = None
                self.is_stuck = False
                self.pos_history.clear()
                return NavigateToPose.Result()
                
            dx = self.current_goal[0] - self.x
            dy = self.current_goal[1] - self.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < 8.0:
                self.get_logger().info('Goal reached. Waiting 1.0s to simulate physical pick-up/drop-off...')
                await rclpy.clock.Clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
                break
                
            await rclpy.clock.Clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            
        goal_handle.succeed()
        self.current_goal = None
        self.is_stuck = False
        self.pos_history.clear()
        
        result = NavigateToPose.Result()
        return result

    def physics_loop(self):
        dt = 0.1
        
        self.x += self.target_vel_x * dt
        self.y += self.target_vel_y * dt
        
        self.target_vel_x *= 0.5
        self.target_vel_y *= 0.5

        if self.current_goal is not None:
            self.battery -= 0.05

        self.ticks += 1
        if self.current_goal is not None and self.ticks % 10 == 0:
            self.pos_history.append((self.x, self.y))
            if len(self.pos_history) > 3:
                self.pos_history.pop(0)
                old_x, old_y = self.pos_history[0]
                dist_moved = math.sqrt((self.x - old_x)**2 + (self.y - old_y)**2)
                if dist_moved < 2.0:
                    self.is_stuck = True
                else:
                    self.is_stuck = False

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = float(self.x)
        pose_msg.pose.position.y = float(self.y)
        self.pose_pub.publish(pose_msg)
        
        if self.current_goal is not None:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = pose_msg.header.stamp
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position.x = float(self.current_goal[0])
            goal_msg.pose.position.y = float(self.current_goal[1])
            self.goal_pub.publish(goal_msg)
            
        scan = LaserScan()
        scan.header.stamp = pose_msg.header.stamp
        scan.header.frame_id = self.robot_id
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2 * math.pi) / 24
        scan.range_min = 0.0
        scan.range_max = 100.0
        
        # Base static chargers
        obstacles = [(50.0, 50.0), (850.0, 650.0), (450.0, 50.0)]
        ranges = []
        
        # Combine static objects and our synthetic dynamic objects for Raycasting
        dynamic_obs = [(obs['x'], obs['y']) for obs in self.dynamic_obstacles]
        all_obstacles = obstacles + dynamic_obs
        
        for i in range(24):
            angle = scan.angle_min + i * scan.angle_increment
            dist = scan.range_max
            for ox, oy in all_obstacles:
                to_obs = math.hypot(ox - self.x, oy - self.y)
                angle_to_obs = math.atan2(oy - self.y, ox - self.x)
                if to_obs < scan.range_max and abs(angle - angle_to_obs) < 0.2:
                    dist = min(dist, to_obs - 5.0) # Small 5m radius for synthetic small objects!
            ranges.append(float(dist))
            
        scan.ranges = ranges
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
