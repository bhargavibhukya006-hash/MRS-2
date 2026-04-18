#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import heapq
import math

class AStarGlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_global_planner_node')
        
        # Core Architecture: Continuous -> Discrete Grid Mapping
        self.world_size = 1000.0  # 1000x1000 continuous space
        self.resolution = 20.0    # Map 20 continuous units exactly to 1 grid cell
        self.grid_size = int(self.world_size / self.resolution)
        
        self.obstacles = set()
        self.generate_static_obstacles()
        
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/astar_map_visuals', 10)
        
        self.timer = self.create_timer(1.0, self.publish_grid_markers)
        
        self.get_logger().info(f"ROS 2 A* Global Planner Active. Operating on {self.grid_size}x{self.grid_size} Costmap.")

    def generate_static_obstacles(self):
        # We can inject static obstacles here. Dynamic obstacles are ignored by A* 
        # (they are left for the RL Local Planner to dodge!)
        chargers = [(50.0, 50.0), (850.0, 650.0), (450.0, 50.0)]
        for cx, cy in chargers:
            i, j = self.to_grid(cx, cy)
            # Add padding around obstacles
            for di in range(-2, 3):
                for dj in range(-2, 3):
                    self.obstacles.add((i+di, j+dj))

    def to_grid(self, x, y):
        i = int(max(0, min(self.world_size-1, x)) / self.resolution)
        j = int(max(0, min(self.world_size-1, y)) / self.resolution)
        return (i, j)

    def to_continuous(self, i, j):
        x = (i * self.resolution) + (self.resolution / 2.0)
        y = (j * self.resolution) + (self.resolution / 2.0)
        return (x, y)

    def heuristic(self, a, b):
        # Euclidean is better for an 8-way movement grid than Manhattan
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        # Support 8-way continuous-like movement instead of strict 4-way Pygame grid
        directions = [(x+1,y), (x-1,y), (x,y+1), (x,y-1), (x+1,y+1), (x-1,y-1), (x+1,y-1), (x-1,y+1)]
        for nx, ny in directions:
            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                if (nx, ny) not in self.obstacles:
                    neighbors.append((nx, ny))
        return neighbors

    def compute_astar_path(self, start_x, start_y, goal_x, goal_y):
        start = self.to_grid(start_x, start_y)
        goal = self.to_grid(goal_x, goal_y)
        
        if start in self.obstacles or goal in self.obstacles:
            self.get_logger().error("A* CRITICAL: Start or Goal is inside a static obstacle!")
            return []
            
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        while open_list:
            _, current = heapq.heappop(open_list)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                
                # Convert Discrete Path back to a Continuous Line of Waypoints!
                continuous_path = [self.to_continuous(n[0], n[1]) for n in path]
                continuous_path.append((goal_x, goal_y)) # ensure precise goal ending
                return continuous_path
                
            for neighbor in self.get_neighbors(current):
                cost = 1.414 if (neighbor[0] != current[0] and neighbor[1] != current[1]) else 1.0
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))
                    
        self.get_logger().warn("A* Search exhausted. No path found.")
        return []

    def publish_grid_markers(self):
        # Visualizes the continuous mapping costmap in RViz
        arr = MarkerArray()
        idx = 0
        for obs in self.obstacles:
            x, y = self.to_continuous(obs[0], obs[1])
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "astar_costmap"
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 2.0
            m.scale.x = self.resolution
            m.scale.y = self.resolution
            m.scale.z = 4.0
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3) # Transparent Red Costmap
            arr.markers.append(m)
            idx += 1
            
        self.marker_pub.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = AStarGlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
