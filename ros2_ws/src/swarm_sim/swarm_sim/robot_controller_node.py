#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import math
import random

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Declare parameters (for namespaces, typically parsed externally, but we'll use node name)
        self.robot_id = self.get_name()
        
        # Continuous state
        self.x = random.uniform(100.0, 800.0)
        self.y = random.uniform(100.0, 600.0)
        self.radius = 12.0
        self.speed = 150.0  # units/sec (pixels/sec in pygame, mm/s in ros depending on scale)
        
        self.battery = 100.0
        
        self.pos_history = []
        self.is_stuck = False
        self.ticks = 0
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.heartbeat_pub = self.create_publisher(String, 'heartbeat', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Action Server for "NavigateToPose" (mimicking Nav2 for simplicity)
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
        
        self.current_goal = None
        self.get_logger().info(f"{self.robot_id} Initialized at ({self.x:.1f}, {self.y:.1f})")

    def heartbeat_timer(self):
        msg = String()
        if self.is_stuck:
            msg.data = f"STATUS_BLOCKED_{self.battery:.1f}"
        else:
            msg.data = f"OK_BATTERY_{self.battery:.1f}"
        self.heartbeat_pub.publish(msg)

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
                
            # Sleep to yield execution in python rclpy async model
            await rclpy.clock.Clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            
        goal_handle.succeed()
        self.current_goal = None
        self.is_stuck = False
        self.pos_history.clear()
        
        result = NavigateToPose.Result()
        return result

    def physics_loop(self):
        # 1. Update Position based on current goal (Proportional Controller)
        if self.current_goal is not None:
            dx = self.current_goal[0] - self.x
            dy = self.current_goal[1] - self.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > 0.1:
                cmd = Twist()
                # Assuming omnidirectional for 1:1 pygame compatibility
                vel_x = (dx / dist) * self.speed
                vel_y = (dy / dist) * self.speed
                
                dt = 0.1 # 10Hz
                self.x += vel_x * dt
                self.y += vel_y * dt
                
                cmd.linear.x = vel_x
                cmd.linear.y = vel_y
                self.cmd_vel_pub.publish(cmd)
                
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

        # 2. Publish Pose (Ground truth sim)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = float(self.x)
        pose_msg.pose.position.y = float(self.y)
        self.pose_pub.publish(pose_msg)

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
