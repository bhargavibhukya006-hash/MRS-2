import pygame
import random
import math
import heapq
from pygame.math import Vector2

# --- CONFIGURATION ---
WIDTH, HEIGHT = 900, 700
FPS = 60

# --- INTERNAL ENUMS ---
STATE_MOVING = 0
STATE_TASK_COMPLETE = 1
STATE_NO_PATH = 2
STATE_RECHARGING = 3
STATE_IDLE = 4
STATE_STUCK = 5

# Colors
BG_COLOR = (25, 25, 30)
BOT_COLOR = (50, 150, 255)
OBS_COLOR = (200, 80, 80)
WAYPOINT_COLOR = (50, 50, 60)
PATH_COLOR = (255, 255, 0)
TARGET_COLOR = (50, 200, 50)
CHARGER_COLOR = (50, 100, 250)
PANEL_COLOR = (15, 15, 20)

# ----------------------------------------
# 1. SPATIAL HASHING & MATH
# ----------------------------------------
class SpatialHash:
    def __init__(self, cell_size=120):
        self.cell_size = cell_size
        self.grid = {}

    def _get_keys(self, pos, radius):
        min_x = int((pos.x - radius) // self.cell_size)
        max_x = int((pos.x + radius) // self.cell_size)
        min_y = int((pos.y - radius) // self.cell_size)
        max_y = int((pos.y + radius) // self.cell_size)
        return [(x, y) for x in range(min_x, max_x + 1) for y in range(min_y, max_y + 1)]

    def insert(self, item, pos, radius=1):
        for key in self._get_keys(pos, radius):
            if key not in self.grid:
                self.grid[key] = []
            self.grid[key].append(item)

    def query(self, pos, radius):
        results = set()
        for key in self._get_keys(pos, radius):
            if key in self.grid:
                results.update(self.grid[key])
        return results

def line_intersects_circle(p1, p2, center, r):
    d = p2 - p1
    f = p1 - center
    a = d.dot(d)
    b = 2 * f.dot(d)
    c = f.dot(f) - r*r
    
    discriminant = b*b - 4*a*c
    if discriminant < 0: return False
        
    discriminant = math.sqrt(discriminant)
    if a == 0: return False
    t1 = (-b - discriminant) / (2*a)
    t2 = (-b + discriminant) / (2*a)
    return (0 <= t1 <= 1) or (0 <= t2 <= 1)

# ----------------------------------------
# 2. A* PATHFINDING & RESERVATION MAP
# ----------------------------------------
def run_a_star(nodes, start_idx, end_idx, end_pos, obstacles, bot_radius):
    distances = {i: float('inf') for i in nodes}
    distances[start_idx] = 0
    previous = {i: None for i in nodes}
    pq = [(0, start_idx)]
    
    while pq:
        current_f, current_idx = heapq.heappop(pq)
        
        if current_idx == end_idx:
            path = []
            while current_idx is not None:
                path.append(current_idx)
                current_idx = previous[current_idx]
            return path[::-1]
            
        current_dist = distances[current_idx]
            
        for neighbor_idx, baseline_dist in nodes[current_idx]['neighbors']:
            neighbor_pos = nodes[neighbor_idx]["pos"]
            
            blocked = False
            for obs in obstacles:
                if line_intersects_circle(nodes[current_idx]["pos"], neighbor_pos, obs.pos, obs.radius + bot_radius):
                    blocked = True
                    break
            if blocked: continue

            h_n = neighbor_pos.distance_to(end_pos)
            congestion_penalty = nodes[neighbor_idx]["reservations"] * 400 
            distance = current_dist + baseline_dist + congestion_penalty
            
            if distance < distances[neighbor_idx]:
                distances[neighbor_idx] = distance
                previous[neighbor_idx] = current_idx
                f_score = distance + h_n
                heapq.heappush(pq, (f_score, neighbor_idx))
    return None

# ----------------------------------------
# 3. ENTITIES
# ----------------------------------------
class DynamicObstacle:
    def __init__(self, pos, radius):
        self.pos = Vector2(pos)
        self.radius = radius
        self.vel = Vector2(random.uniform(-0.8, 0.8), random.uniform(-0.8, 0.8))

    def update(self):
        self.pos += self.vel
        if self.pos.x - self.radius < 0 or self.pos.x + self.radius > WIDTH: self.vel.x *= -1
        if self.pos.y - self.radius < 0 or self.pos.y + self.radius > HEIGHT: self.vel.y *= -1

class Bot:
    def __init__(self, id, pos):
        self.id = id
        self.pos = Vector2(pos)
        self.radius = 12
        self.speed = 3.5
        
        # --- STATE AND LOGISTICS ---
        self.state = STATE_IDLE
        self.battery = 100.0
        self.recent_replans = 0
        self.frames_idle = 0
        self.pos_history = []
        
        self.target = None
        self.path_nodes = []
        self.path_vecs = []
        self.nav_nodes_ref = None 
        
    def set_target(self, target_vec, nav_graph, obstacles, new_state=STATE_MOVING):
        self.target = target_vec
        self.nav_nodes_ref = nav_graph
        self.state = new_state
        self.recent_replans = 0
        self.calculate_path(nav_graph, obstacles)
        
    def calculate_path(self, nav_graph, obstacles):
        for nid in self.path_nodes:
            if str(nid).isdigit() and nid in nav_graph:
                nav_graph[nid]["reservations"] = max(0, nav_graph[nid]["reservations"] - 1)

        temp_nodes = {k: {"pos": v["pos"], "neighbors": list(v["neighbors"]), "reservations": v["reservations"]} for k, v in nav_graph.items()}
        start_id = "S"
        end_id = "E"
        temp_nodes[start_id] = {"pos": self.pos, "neighbors": [], "reservations": 0}
        temp_nodes[end_id] = {"pos": self.target, "neighbors": [], "reservations": 0}
        
        for nid, node in nav_graph.items():
            if not any(line_intersects_circle(self.pos, node["pos"], obs.pos, obs.radius + self.radius) for obs in obstacles):
                temp_nodes[start_id]["neighbors"].append((nid, self.pos.distance_to(node["pos"])))
            if not any(line_intersects_circle(self.target, node["pos"], obs.pos, obs.radius + self.radius) for obs in obstacles):
                temp_nodes[nid]["neighbors"].append((end_id, self.target.distance_to(node["pos"])))

        route = run_a_star(temp_nodes, start_id, end_id, self.target, obstacles, self.radius)
        if route:
            self.path_nodes = route[1:-1]
            self.path_vecs = [temp_nodes[r]["pos"] for r in route[1:]]
            for nid in self.path_nodes:
                nav_graph[nid]["reservations"] += 1
        else:
            self.path_vecs = []
            self.path_nodes = []
            
        self.recent_replans += 1

    def update(self, obstacles, metrics, chargers):
        # 1. BATTERY DRAINAGE LOGIC
        if self.state in [STATE_MOVING, STATE_RECHARGING] and len(self.path_vecs) > 0:
            self.battery -= 0.05
        
        # 2. EMERGENCY CHARGING ROUTER
        if self.battery <= 20.0 and self.state == STATE_MOVING:
            # Abandon job to survive!
            closest_charger = min(chargers, key=lambda c: self.pos.distance_to(c))
            self.set_target(closest_charger, self.nav_nodes_ref, obstacles, new_state=STATE_RECHARGING)
            
        # 3. IDLING/CHARGING LOGIC
        if not self.path_vecs: 
            if self.state == STATE_RECHARGING:
                self.battery = min(100.0, self.battery + 1.0) # Docked, charging very fast
                if self.battery >= 100.0:
                    return STATE_IDLE # Done charging, needs a new job
                return STATE_RECHARGING
            return STATE_NO_PATH if self.target else STATE_IDLE 
            
        # 4. STUCK DETECTION LOGIC (Warehouse fail-safe)
        self.pos_history.append(Vector2(self.pos))
        if len(self.pos_history) > 60:
            self.pos_history.pop(0)
            
        # If we barely moved over 60 frames OR we replanned completely out of control
        if len(self.pos_history) == 60:
            dist_moved = self.pos.distance_to(self.pos_history[0])
            if dist_moved < 5.0 or self.recent_replans > 8:
                self.pos_history.clear()
                self.recent_replans = 0
                return STATE_STUCK  # Fire the fail-safe
        
        # 5. PREDICTIVE TRAJECTORY AVOIDANCE
        next_wp = self.path_vecs[0]
        future_blocked = False
        for obs in obstacles:
            projected_obs = obs.pos + (obs.vel * 40)
            if line_intersects_circle(self.pos, next_wp, projected_obs, obs.radius + self.radius):
                future_blocked = True
                break
                
        if future_blocked:
            metrics["replans"] += 1
            self.calculate_path(self.nav_nodes_ref, obstacles)
            if not self.path_vecs: return STATE_NO_PATH
            next_wp = self.path_vecs[0]

        # 6. PHYSICAL MOVEMENT
        vec_to_wp = next_wp - self.pos
        dist = vec_to_wp.length()
        drift = Vector2(random.uniform(-0.4, 0.4), random.uniform(-0.4, 0.4))
        
        if dist < self.speed:
            self.pos = next_wp
            self.path_vecs.pop(0)
            if self.path_nodes:
                passed_nid = self.path_nodes.pop(0)
                if passed_nid in self.nav_nodes_ref:
                    self.nav_nodes_ref[passed_nid]["reservations"] = max(0, self.nav_nodes_ref[passed_nid]["reservations"] - 1)
                    
            if not self.path_vecs:
                if self.state == STATE_RECHARGING:
                    return STATE_RECHARGING # Just arrived at docking station
                return STATE_TASK_COMPLETE
        else:
            move_vec = (vec_to_wp.normalize() * self.speed) + drift
            self.pos += move_vec
            metrics["distance_travelled"] += move_vec.length()
            
        return self.state

# ----------------------------------------
# 4. ENVIRONMENT & TASK MANAGER 
# ----------------------------------------
class SwarmSimulator:
    def __init__(self):
        self.obstacles = []
        self.nav_nodes = {}
        self.chargers = [Vector2(50, 50), Vector2(WIDTH-50, HEIGHT-50), Vector2(WIDTH//2, 50)]
        self.spatial_hash = SpatialHash(cell_size=150)
        self.bots = [Bot(i, Vector2(WIDTH//2, HEIGHT//2)) for i in range(8)]
        
        self.metrics = {
            "tasks_completed": 0,
            "replans": 0,
            "stuck_recoveries": 0,
            "collisions_prevented": 0,
            "distance_travelled": 0.0
        }
        
        self.show_prm = False
        self.show_vectors = True
        self.show_radii = False
        self.show_heatmap = True

        self.setup_random_field()
        for bot in self.bots:
            self.assign_new_task(bot)
            
    def assign_new_task(self, bot):
        tgt = Vector2(random.randint(100, WIDTH-100), random.randint(100, HEIGHT-100))
        bot.set_target(tgt, self.nav_nodes, self.obstacles, new_state=STATE_MOVING)

    def setup_random_field(self):
        for _ in range(12):
            r = random.randint(35, 75)
            pos = (random.randint(200, WIDTH-200), random.randint(r, HEIGHT-r))
            self.obstacles.append(DynamicObstacle(pos, r))
                    
        valid_points = []
        for _ in range(700): 
            pt = Vector2(random.randint(20, WIDTH-20), random.randint(20, HEIGHT-20))
            if all(pt.distance_to(obs.pos) > obs.radius + 15 for obs in self.obstacles):
                valid_points.append(pt)
                self.spatial_hash.insert(len(valid_points)-1, pt, 0)
                
        for i, p1 in enumerate(valid_points):
            neighbors = []
            nearby_indices = self.spatial_hash.query(p1, 130)
            
            for j in nearby_indices:
                if i == j: continue
                p2 = valid_points[j]
                dist = p1.distance_to(p2)
                if dist < 120: 
                    if not any(line_intersects_circle(p1, p2, obs.pos, obs.radius + 15) for obs in self.obstacles):
                        neighbors.append((j, dist))
            self.nav_nodes[i] = {"pos": p1, "neighbors": neighbors, "reservations": 0}

    def resolve_bot_collisions(self):
        for i, b1 in enumerate(self.bots):
            for b2 in self.bots[i+1:]:
                dist = b1.pos.distance_to(b2.pos)
                min_dist = b1.radius + b2.radius + 6 
                if dist < min_dist:
                    self.metrics["collisions_prevented"] += 1
                    overlap = min_dist - dist
                    push_dir = Vector2(1,0) if dist == 0 else (b1.pos - b2.pos).normalize()
                    b1.pos += push_dir * (overlap / 2)
                    b2.pos -= push_dir * (overlap / 2)

    def draw_dashboard(self, surface):
        pygame.draw.rect(surface, PANEL_COLOR, (10, 10, 310, 240), border_radius=5)
        font = pygame.font.SysFont("consolas", 14)
        
        charging_cnt = sum(1 for b in self.bots if b.state == STATE_RECHARGING)
        
        stats = [
            "  --- FLEET METRICS DASHBOARD ---",
            f"Tasks Completed:      {self.metrics['tasks_completed']}",
            f"A* Replans Evoked:    {self.metrics['replans']}",
            f"Stuck Recoveries:     {self.metrics['stuck_recoveries']}",
            f"Distance Traveled:    {int(self.metrics['distance_travelled'])} units",
            f"Active Chargers Usage:{charging_cnt} Bots",
            f"Shared Cost Load:     {sum(1 for n in self.nav_nodes.values() if n['reservations']>0)} paths",
            "",
            "[1] NavMesh  [2] Vectors",
            "[3] Radii    [4] Heatmap"
        ]
        
        y = 20
        for line in stats:
            text = font.render(line, True, (200, 255, 220))
            if "---" in line: text = font.render(line, True, (255, 200, 50))
            if "[" in line: text = font.render(line, True, (150, 150, 150))
            if "Stuck Recoveries" in line and self.metrics["stuck_recoveries"] > 0: text = font.render(line, True, (255, 100, 100))
            surface.blit(text, (20, y))
            y += 20

    def draw(self, surface):
        surface.fill(BG_COLOR)
        
        if self.show_prm:
            for nid, node in self.nav_nodes.items():
                p1 = node["pos"]
                for neighbor_id, dist in node["neighbors"]:
                    p2 = self.nav_nodes[neighbor_id]["pos"]
                    pygame.draw.line(surface, WAYPOINT_COLOR, (p1.x, p1.y), (p2.x, p2.y), 1)
        
        if self.show_heatmap:
            for nid, node in self.nav_nodes.items():
                res = node["reservations"]
                if res > 0:
                    rad = 8 + (res * 5)
                    color = (min(255, 30 + res*60), 40, 40)
                    pygame.draw.circle(surface, color, (int(node["pos"].x), int(node["pos"].y)), rad, 0)

        # Draw Chargers
        for char in self.chargers:
            pygame.draw.rect(surface, CHARGER_COLOR, (int(char.x-15), int(char.y-15), 30, 30), border_radius=4)
            pygame.draw.rect(surface, (255,255,255), (int(char.x-15), int(char.y-15), 30, 30), 2, border_radius=4)

        for obs in self.obstacles:
            pygame.draw.circle(surface, OBS_COLOR, (int(obs.pos.x), int(obs.pos.y)), int(obs.radius))
            
        for bot in self.bots:
            # Draw bot logic rendering
            bot_color = BOT_COLOR
            if bot.state == STATE_RECHARGING: bot_color = (255, 255, 0) # Flash yellow if trying to charge
            
            if bot.target and bot.state == STATE_MOVING:
                pygame.draw.circle(surface, TARGET_COLOR, (int(bot.target.x), int(bot.target.y)), 10, 2)
                if len(bot.path_vecs) > 0:
                    pts = [bot.pos] + bot.path_vecs
                    pygame.draw.lines(surface, PATH_COLOR, False, [(p.x, p.y) for p in pts], 2)
                    
            if self.show_radii:
                pygame.draw.circle(surface, (255,255,255), (int(bot.pos.x), int(bot.pos.y)), int(bot.radius + 6), 1)

            pygame.draw.circle(surface, bot_color, (int(bot.pos.x), int(bot.pos.y)), int(bot.radius))
            
            # Draw battery bar
            bat_pct = bot.battery / 100.0
            pygame.draw.rect(surface, (255,0,0), (int(bot.pos.x)-10, int(bot.pos.y)-20, 20, 4))
            pygame.draw.rect(surface, (0,255,0), (int(bot.pos.x)-10, int(bot.pos.y)-20, 20 * bat_pct, 4))

            if self.show_vectors and len(bot.path_vecs) > 0:
                vec = (bot.path_vecs[0] - bot.pos).normalize() * 30
                pygame.draw.line(surface, (0, 255, 255), (bot.pos.x, bot.pos.y), (bot.pos.x + vec.x, bot.pos.y + vec.y), 3)

        self.draw_dashboard(surface)

# ==========================================
# 5. EXECUTION BOOTSTRAP
# ==========================================
if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("A* Warehouse Simulator - Logistics Edition")
    clock = pygame.time.Clock()
    
    sim = SwarmSimulator()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1: sim.show_prm = not sim.show_prm
                if event.key == pygame.K_2: sim.show_vectors = not sim.show_vectors
                if event.key == pygame.K_3: sim.show_radii = not sim.show_radii
                if event.key == pygame.K_4: sim.show_heatmap = not sim.show_heatmap

        for obs in sim.obstacles:
            obs.update()
            
        for bot in sim.bots:
            new_state = bot.update(sim.obstacles, sim.metrics, sim.chargers)
            
            if new_state == STATE_TASK_COMPLETE:
                sim.metrics["tasks_completed"] += 1
                sim.assign_new_task(bot)
                
            elif new_state == STATE_STUCK:
                sim.metrics["stuck_recoveries"] += 1
                print(f"[ERROR] Bot {bot.id} stuck. Abandoning task and forcing escape via Replan.")
                sim.assign_new_task(bot)
                
            elif new_state == STATE_IDLE:
                sim.assign_new_task(bot)
            
        sim.resolve_bot_collisions()
            
        sim.draw(screen)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
