#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import math
import json

class Task:
    def __init__(self, task_id, start_x, start_y, end_x, end_y):
        self.id = task_id
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y
        self.picked_up = False
        self.completed = False
        self.current_x = start_x
        self.current_y = start_y

class FleetManagerNode(Node):
    def __init__(self, num_bots=5):
        super().__init__('fleet_manager_node')
        self.num_bots = num_bots
        
        # New Courier Style Tasks (Start -> End)
        self.pending_tasks = [
            Task("T1", 100.0, 100.0, 750.0, 600.0),
            Task("T2", 200.0, 500.0, 800.0, 150.0),
            Task("T3", 700.0, 100.0, 150.0, 450.0),
            Task("T4", 400.0, 50.0, 400.0, 600.0)
        ]
        
        self.bots = {
            f"robot_{i}": {
                "status": "ACTIVE",
                "task": None,
                "pos": (0.0, 0.0),
                "last_heartbeat": self.get_clock().now().nanoseconds / 1e9,
                "action_client": ActionClient(self, NavigateToPose, f'/robot_{i}/navigate_to_pose')
            }
            for i in range(num_bots)
        }
        
        self.metrics = {
            "tasks_completed": 0,
            "reassignments_made": 0
        }

        # Subscribers for heartbeats and positions
        for i in range(self.num_bots):
            bot_id = f"robot_{i}"
            self.create_subscription(String, f'/{bot_id}/heartbeat', lambda msg, bid=bot_id: self.heartbeat_callback(msg, bid), 10)
            self.create_subscription(PoseStamped, f'/{bot_id}/pose', lambda msg, bid=bot_id: self.pose_callback(msg, bid), 10)
            
        # Publisher for Fleet Status (used by the visualizer)
        self.status_pub = self.create_publisher(String, '/fleet_status', 10)
        
        self.timer = self.create_timer(1.0, self.monitor_fleet)
        self.status_timer = self.create_timer(0.5, self.publish_fleet_status)
        
        self.get_logger().info("Fleet Manager Initialized. Assigning Courier Tasks...")
        self.assign_tasks()

    def heartbeat_callback(self, msg, bot_id):
        if bot_id in self.bots:
            self.bots[bot_id]["last_heartbeat"] = self.get_clock().now().nanoseconds / 1e9
            
            if self.bots[bot_id]["status"] == "OFFLINE" and "OK" in msg.data:
                self.get_logger().info(f"[RECOVERY] {bot_id} is back online.")
                self.bots[bot_id]["status"] = "ACTIVE"
                
            if "STATUS_BLOCKED" in msg.data:
                if self.bots[bot_id]["status"] != "BLOCKED":
                    self.get_logger().error(f"[!!!] OBSTACLE EVENT: {bot_id} is STUCK! Pulling task. [!!!]")
                    self.bots[bot_id]["status"] = "BLOCKED"
                    stuck_task = self.bots[bot_id]["task"]
                    if stuck_task:
                        self.get_logger().info(f"Retrieving blocked task: {stuck_task.id}")
                        self.bots[bot_id]["task"] = None
                        self.reassign_failed_task(stuck_task)
            elif "OK" in msg.data and self.bots[bot_id]["status"] == "BLOCKED":
                self.bots[bot_id]["status"] = "ACTIVE"

    def pose_callback(self, msg, bot_id):
        if bot_id in self.bots:
            self.bots[bot_id]["pos"] = (msg.pose.position.x, msg.pose.position.y)
            # Update item's current location if carrying
            task = self.bots[bot_id]["task"]
            if task and task.picked_up and not task.completed:
                task.current_x = msg.pose.position.x
                task.current_y = msg.pose.position.y

    def publish_fleet_status(self):
        carrying = []
        for bid, b in self.bots.items():
            if b["task"] and b["task"].picked_up:
                carrying.append(bid)
        msg = String()
        msg.data = json.dumps({"carrying_bots": carrying})
        self.status_pub.publish(msg)

    def monitor_fleet(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        for bot_id, bot in self.bots.items():
            if bot["status"] == "ACTIVE" and (current_time - bot["last_heartbeat"]) > 3.0:
                self.get_logger().error(f"[!!!] CRITICAL EVENT: {bot_id} crashed. [!!!]")
                bot["status"] = "OFFLINE"
                stolen_task = bot["task"]
                if stolen_task:
                    self.get_logger().info(f"Retrieving lost task: {stolen_task.id}")
                    bot["task"] = None
                    self.reassign_failed_task(stolen_task)
                
    def assign_tasks(self):
        for bot_id, bot in self.bots.items():
            if bot["status"] == "ACTIVE" and bot["task"] is None and self.pending_tasks:
                task = self.pending_tasks.pop(0)
                bot["task"] = task
                self.get_logger().info(f"{bot_id} initially assigned: {task.id}")
                self.send_task_goal(bot_id, task)

    def reassign_failed_task(self, failed_task):
        self.metrics["reassignments_made"] += 1
        
        best_bot_id = None
        best_score = float('inf')  

        active_bots = {k: v for k, v in self.bots.items() if v["status"] == "ACTIVE"}
        if not active_bots:
            self.get_logger().error("CRITICAL: No active bots alive to take the task!")
            self.pending_tasks.append(failed_task)
            return

        target_x = failed_task.current_x if failed_task.picked_up else failed_task.start_x
        target_y = failed_task.current_y if failed_task.picked_up else failed_task.start_y

        for bot_id, bot in active_bots.items():
            dx = bot["pos"][0] - target_x
            dy = bot["pos"][1] - target_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            # Simplified workload logic: penalty if already busy
            workload_penalty = 1000 if bot["task"] is not None else 0
            score = dist + workload_penalty
            if score < best_score:
                best_score = score
                best_bot_id = bot_id

        assigned_to = best_bot_id
        if assigned_to is not None:
            # If the bot is already doing something, queue it
            if self.bots[assigned_to]["task"] is None:
                self.bots[assigned_to]["task"] = failed_task
                self.send_task_goal(assigned_to, failed_task)
                self.get_logger().info(f"-> SMART REASSIGN: Task {failed_task.id} given to {assigned_to}!")
            else:
                self.pending_tasks.insert(0, failed_task)
                self.get_logger().info(f"-> Task {failed_task.id} pushed top of queue.")

    def send_task_goal(self, bot_id, task):
        client = self.bots[bot_id]["action_client"]
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f"Action server for {bot_id} not available.")
            return
            
        goal_msg = NavigateToPose.Goal()
        if not task.picked_up:
            goal_msg.pose.pose.position.x = task.current_x
            goal_msg.pose.pose.position.y = task.current_y
            self.get_logger().info(f"Sending goal to {bot_id}: PICKUP -> ({task.current_x}, {task.current_y})")
        else:
            goal_msg.pose.pose.position.x = task.end_x
            goal_msg.pose.pose.position.y = task.end_y
            self.get_logger().info(f"Sending goal to {bot_id}: DELIVER -> ({task.end_x}, {task.end_y})")
            
        goal_msg.pose.header.frame_id = "map"
        
        send_goal_future = client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future, bid=bot_id, t=task: self.goal_response_callback(future, bid, t))

    def goal_response_callback(self, future, bot_id, task):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f"Goal {task.id} rejected by {bot_id}")
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda fut, bid=bot_id, t=task: self.get_result_callback(fut, bid, t))

    def get_result_callback(self, future, bot_id, task):
        # We arrived at the goal
        if not task.picked_up:
            self.get_logger().info(f"[!] {bot_id} PICKED UP task {task.id}!")
            task.picked_up = True
            self.send_task_goal(bot_id, task)
        else:
            self.metrics["tasks_completed"] += 1
            self.get_logger().info(f"[!] {bot_id} COMPLETED delivery of task {task.id}!")
            self.bots[bot_id]["task"] = None
            self.assign_tasks()

def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
