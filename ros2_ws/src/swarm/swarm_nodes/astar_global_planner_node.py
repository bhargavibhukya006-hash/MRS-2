#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import heapq
import math
import time

class AStarGlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_global_planner_node')
        
        self.world_size = 1000.0  
        self.resolution = 5.0    
        self.grid_size = int(self.world_size / self.resolution)
        
        # Maps purely static features (walls, chargers)
        self.static_obstacles = set()
        self.static_inflated_zone = set()
        self.generate_static_obstacles()
        
        # Transient Dynamic Obstacles Dictionary: { (x, y) : timestamp }
        self.dynamic_obstacles = {}
        
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/astar_map_visuals', 10)
        
        self.current_pose = None
        self.current_goal = None
        
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10) 
        self.create_subscription(Point, 'add_dynamic_obstacle', self.dynamic_obstacle_callback, 10)
        
        self.create_timer(3.0, self.publish_grid_markers)
        self.create_timer(2.0, self.spawn_random_obstacles) # Spawn random dynamic obstacles
        self.get_logger().info(f"Layer 1: A* Global Planner Active. Operating continuously on {self.grid_size}x{self.grid_size} Map.")

    def pose_callback(self, msg):
        self.current_pose = (msg.pose.position.x, msg.pose.position.y)

    def goal_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().warn("A* received goal but no current pose yet.")
            return
            
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"New Global Route Requested to {self.current_goal}")
        self.trigger_replan()

    def dynamic_obstacle_callback(self, msg):
        self.get_logger().warn(f"Layer 1: Received Dynamic Obstacle at ({msg.x:.1f}, {msg.y:.1f}). Mapping Temporarily...")
        block_radius = 0 # Small Box (1x1 grid cell = 5x5m bounding block)
        center_i, center_j = self.to_grid(msg.x, msg.y)
        
        stamp = time.time()
        for di in range(-block_radius, block_radius+1):
            for dj in range(-block_radius, block_radius+1):
                nx, ny = center_i + di, center_j + dj
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    self.dynamic_obstacles[(nx, ny)] = stamp
                    
        self.publish_grid_markers()
        self.trigger_replan()

    def cleanup_dynamic_obstacles(self):
        # Allow dynamic obstacles to vanish after 10.0 seconds
        current_time = time.time()
        expired = [k for k, v in self.dynamic_obstacles.items() if current_time - v > 10.0]
        for k in expired:
            del self.dynamic_obstacles[k]

    def spawn_random_obstacles(self):
        # Randomly spawn small dynamic obstacles purely in the planner map to satisfy requirement
        import random
        # Only spawn if less than 20 obstacles on map
        if len(self.dynamic_obstacles) < 20:
            num_to_spawn = random.randint(1, 3)
            current_time = time.time()
            for _ in range(num_to_spawn):
                # Random coordinates
                rx = random.uniform(50.0, 950.0)
                ry = random.uniform(50.0, 950.0)
                idx, jdx = self.to_grid(rx, ry)
                
                # Make them 1x1 block
                if (idx, jdx) not in self.static_obstacles:
                    # Give them random expiry times (e.g. 5 to 15 seconds)
                    self.dynamic_obstacles[(idx, jdx)] = current_time + random.uniform(-5.0, 5.0) 
        
        self.cleanup_dynamic_obstacles()
        self.publish_grid_markers()

    def trigger_replan(self):
        if self.current_pose is None or self.current_goal is None:
            return
            
        path_wps = self.compute_astar_path(self.current_pose[0], self.current_pose[1], 
                                           self.current_goal[0], self.current_goal[1])
        if path_wps:
            self.publish_ros_path(path_wps)
        else:
            self.get_logger().error("Triggered Replan Failed! No Route found.")

    def generate_static_obstacles(self):
        chargers = [(50.0, 50.0), (850.0, 650.0), (450.0, 50.0)]
        for cx, cy in chargers:
            i, j = self.to_grid(cx, cy)
            for di in range(-4, 5):
                for dj in range(-4, 5):
                    nx, ny = i + di, j + dj
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        self.static_obstacles.add((nx, ny))
                        
        for obs in list(self.static_obstacles): 
            for di in range(-8, 9):
                for dj in range(-8, 9):
                    nx, ny = obs[0] + di, obs[1] + dj
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        if (nx, ny) not in self.static_obstacles:
                            self.static_inflated_zone.add((nx, ny))

    def to_grid(self, x, y):
        i = int(max(0, min(self.world_size-1, x)) / self.resolution)
        j = int(max(0, min(self.world_size-1, y)) / self.resolution)
        return (i, j)

    def to_continuous(self, i, j):
        x = (i * self.resolution) + (self.resolution / 2.0)
        y = (j * self.resolution) + (self.resolution / 2.0)
        return (x, y)

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])
        
    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        directions = [(x+1,y), (x-1,y), (x,y+1), (x,y-1), (x+1,y+1), (x-1,y-1), (x+1,y-1), (x-1,y+1)]
        for nx, ny in directions:
            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                if (nx, ny) not in self.static_obstacles and (nx, ny) not in self.dynamic_obstacles:
                    neighbors.append((nx, ny))
        return neighbors

    def compute_astar_path(self, start_x, start_y, goal_x, goal_y):
        self.cleanup_dynamic_obstacles() # Self-Healing Map Routine
        
        start = self.to_grid(start_x, start_y)
        goal = self.to_grid(goal_x, goal_y)
        
        if start in self.static_obstacles or start in self.dynamic_obstacles:
            self.get_logger().error("A* CRITICAL: Start is inside an obstacle!")
            return []
        if goal in self.static_obstacles or goal in self.dynamic_obstacles:
            self.get_logger().error("A* CRITICAL: Goal is inside an obstacle!")
            return []
            
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        closed_set = set() # Standard A* Closed Set Implementation
        
        while open_list:
            _, current = heapq.heappop(open_list)
            
            if current in closed_set:
                continue
            closed_set.add(current)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                
                continuous_path = [self.to_continuous(n[0], n[1]) for n in path]
                continuous_path.append((goal_x, goal_y)) 
                return continuous_path
                
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                    
                cost = 1.414 if (neighbor[0] != current[0] and neighbor[1] != current[1]) else 1.0
                
                if neighbor in self.static_inflated_zone: # Apply soft penalty near static walls
                    cost += 2.0
                    
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))
                    
        self.get_logger().warn("A* Search exhausted. No route found.")
        return []

    def publish_ros_path(self, continuous_waypoints):
        if not continuous_waypoints:
            return
            
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for wpx, wpy in continuous_waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(wpx)
            pose.pose.position.y = float(wpy)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published Layer 1 Route with {len(path_msg.poses)} waypoints.")

    def publish_grid_markers(self):
        arr = MarkerArray()
        idx = 0
        
        # Publish Static Obstacles (Red)
        for obs in self.static_obstacles:
            x, y = self.to_continuous(obs[0], obs[1])
            m = self.create_marker(x, y, idx, 1.0, 0.0, 0.0)
            arr.markers.append(m)
            idx += 1
            
        # Publish Dynamic Obstacles (Yellow)
        self.cleanup_dynamic_obstacles()
        for obs in self.dynamic_obstacles.keys():
            x, y = self.to_continuous(obs[0], obs[1])
            m = self.create_marker(x, y, idx, 1.0, 1.0, 0.0)
            arr.markers.append(m)
            idx += 1
            
        self.marker_pub.publish(arr)

    def create_marker(self, x, y, idx, r, g, b):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "astar_costmap"
        m.id = idx
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 1.0
        m.scale.x = self.resolution
        m.scale.y = self.resolution
        m.scale.z = 2.0
        m.color = ColorRGBA(r=r, g=g, b=b, a=0.5) 
        return m

def main(args=None):
    rclpy.init(args=args)
    node = AStarGlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
