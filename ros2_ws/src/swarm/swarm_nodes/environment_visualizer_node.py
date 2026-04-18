#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA, String
import json

class EnvironmentVisualizerNode(Node):
    def __init__(self):
        super().__init__('environment_visualizer_node')
        
        self.marker_pub = self.create_publisher(MarkerArray, 'env_visuals', 10)
        self.create_subscription(String, '/fleet_status', self.status_callback, 10)
        
        self.chargers = [(50.0, 50.0), (850.0, 650.0), (450.0, 50.0)]
        self.carrying_bots = []
        
        # Subscribe to robot poses internally to draw them in RViz
        self.num_bots = 5
        self.bot_poses = {}
        for i in range(self.num_bots):
            bot_id = f"robot_{i}"
            self.bot_poses[bot_id] = (0.0, 0.0)
            self.create_subscription(PoseStamped, f'/{bot_id}/pose', lambda msg, bid=bot_id: self.pose_callback(msg, bid), 10)

        self.timer = self.create_timer(0.1, self.publish_visuals) # 10Hz redraw
        self.get_logger().info("Environment Visualizer Initialized.")

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.carrying_bots = data.get("carrying_bots", [])
        except:
            pass
            
    def pose_callback(self, msg, bot_id):
        self.bot_poses[bot_id] = (msg.pose.position.x, msg.pose.position.y)

    def publish_visuals(self):
        arr = MarkerArray()
        
        # 1. Chargers
        for i, chg in enumerate(self.chargers):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "chargers"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = chg[0]
            m.pose.position.y = chg[1]
            m.pose.position.z = -1.0
            m.scale.x = 40.0
            m.scale.y = 40.0
            m.scale.z = 2.0
            m.color.a = 0.5
            m.color.r = 0.2
            m.color.g = 0.4
            m.color.b = 0.98
            arr.markers.append(m)
            
        # 2. Robots
        for i in range(self.num_bots):
            bot_id = f"robot_{i}"
            pos = self.bot_poses[bot_id]
            is_carrying = bot_id in self.carrying_bots
            
            mrk = Marker()
            mrk.header.frame_id = "map"
            mrk.header.stamp = self.get_clock().now().to_msg()
            mrk.ns = "robots"
            mrk.id = i
            mrk.type = Marker.CYLINDER
            mrk.action = Marker.ADD
            mrk.pose.position.x = pos[0]
            mrk.pose.position.y = pos[1]
            mrk.pose.position.z = 5.0
            mrk.scale.x = 24.0 # 12 radius * 2
            mrk.scale.y = 24.0
            mrk.scale.z = 5.0
            mrk.color.a = 1.0
            
            if is_carrying:
                # Golden Ring logic
                mrk.color.r = 1.0
                mrk.color.g = 0.84
                mrk.color.b = 0.0
            else:
                mrk.color.r = 0.2
                mrk.color.g = 0.6
                mrk.color.b = 1.0
                
            arr.markers.append(mrk)
            
        self.marker_pub.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
