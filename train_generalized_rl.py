import random
import pygame
import sys

# ==========================================
# 1. THE RELATIVE ENVIRONMENT
# ==========================================
class DynamicObstacleEnv:
    def __init__(self, grid_size=10, num_obstacles=5, render_mode=True):
        self.grid_size = grid_size
        self.num_obstacles = num_obstacles
        self.render_mode = render_mode
        self.cell_size = 60
        
        if self.render_mode:
            pygame.init()
            self.screen_size = self.grid_size * self.cell_size
            self.screen = pygame.display.set_mode((self.screen_size, self.screen_size))
            pygame.display.set_caption("Generalized RL with Moving Obstacles")
            self.clock = pygame.time.Clock()
            self.font = pygame.font.SysFont("Arial", 16)
            
        self.reset()
        
    def reset(self):
        # Spawns agent at top-left, target at bottom-right
        self.agent_pos = [0, 0]
        self.target_pos = [self.grid_size - 1, self.grid_size - 1]
        self.visited_positions = {tuple(self.agent_pos)} # Track where we've been
        
        # DOMAIN RANDOMIZATION: Randomly spawn obstacles every episode
        self.obstacles = []
        while len(self.obstacles) < self.num_obstacles:
            obs = [random.randint(0, self.grid_size - 1), random.randint(0, self.grid_size - 1)]
            # Don't place obstacle on agent or target
            if obs != self.agent_pos and obs != self.target_pos and obs not in self.obstacles:
                self.obstacles.append(obs)
                
        return self.get_relative_state()
        
    def get_relative_state(self):
        # ------------------------------------------------------------------
        # CRITICAL RL CONCEPT: RELATIVE LOCAL OBSERVATION ("Lidar" sensing)
        # ------------------------------------------------------------------
        dx_raw = self.target_pos[0] - self.agent_pos[0]
        dy_raw = self.target_pos[1] - self.agent_pos[1]
        
        # Unrounded raw float normalized distances
        dx_norm = dx_raw / self.grid_size
        dy_norm = dy_raw / self.grid_size
        
        x, y = self.agent_pos
        
        def is_blocked(cx, cy):
            return (cx < 0 or cx >= self.grid_size or cy < 0 or cy >= self.grid_size or [cx, cy] in self.obstacles)

        w_up         = is_blocked(x - 1, y)
        w_down       = is_blocked(x + 1, y)
        w_left       = is_blocked(x, y - 1)
        w_right      = is_blocked(x, y + 1)
        w_up_left    = is_blocked(x - 1, y - 1)
        w_up_right   = is_blocked(x - 1, y + 1)
        w_down_left  = is_blocked(x + 1, y - 1)
        w_down_right = is_blocked(x + 1, y + 1)
        
        return (dx_norm, dy_norm, w_up, w_down, w_left, w_right, w_up_left, w_up_right, w_down_left, w_down_right)

    def step(self, action):
        # --- 1. AGENT MOVES FIRST ---
        # Actions: 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
        new_x, new_y = self.agent_pos[0], self.agent_pos[1]
        
        if action == 0: new_x -= 1
        elif action == 1: new_x += 1
        elif action == 2: new_y -= 1
        elif action == 3: new_y += 1
            
        # Optional: Agent boundary/wall hit check
        if new_x < 0 or new_x >= self.grid_size or new_y < 0 or new_y >= self.grid_size:
            return self.get_relative_state(), -10, False  # Punish Wall hit
            
        if [new_x, new_y] in self.obstacles:
            return self.get_relative_state(), -10, False  # Punish static Obstacle hit
            
        # Agent successfully moves
        self.agent_pos = [new_x, new_y]
        
        # --- 2. ASSESS AGENT POSITION ---
        reward = -1
        done = False
        
        if self.agent_pos == self.target_pos:
            return self.get_relative_state(), 100, True   # Reward Goal
            
        # Penalize Standing Still / Re-visiting cells (Anti-oscillation)
        if tuple(self.agent_pos) in self.visited_positions:
            reward -= 2
        else:
            self.visited_positions.add(tuple(self.agent_pos))

        # --- 3. OBSTACLES MOVE ---
        for i in range(len(self.obstacles)):
            if random.random() < 0.25:  # 25% chance to move each step
                move_dir = random.choice([(0,1), (0,-1), (1,0), (-1,0)])
                new_ox = self.obstacles[i][0] + move_dir[0]
                new_oy = self.obstacles[i][1] + move_dir[1]
                
                # Check valid move
                if (0 <= new_ox < self.grid_size and 0 <= new_oy < self.grid_size and 
                    [new_ox, new_oy] != self.target_pos and [new_ox, new_oy] not in self.obstacles):
                    self.obstacles[i] = [new_ox, new_oy]
                    
                    # If obstacle shifts onto agent's new position!
                    if self.obstacles[i] == self.agent_pos:
                        reward -= 10
            
        return self.get_relative_state(), reward, done

    def render(self, episode, steps, epsilon, is_test=False):
        if not self.render_mode:
            return
            
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        self.screen.fill((20, 20, 20)) # Dark background
        
        # Draw Grid
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                rect = pygame.Rect(y * self.cell_size, x * self.cell_size, self.cell_size, self.cell_size)
                pygame.draw.rect(self.screen, (50, 50, 50), rect, 1)
                
        # Draw target (Green)
        t_rect = pygame.Rect(self.target_pos[1] * self.cell_size, self.target_pos[0] * self.cell_size, self.cell_size, self.cell_size)
        pygame.draw.rect(self.screen, (0, 255, 100), t_rect)
        
        # Draw obstacles (Red)
        for obs in self.obstacles:
            o_rect = pygame.Rect(obs[1] * self.cell_size, obs[0] * self.cell_size, self.cell_size, self.cell_size)
            pygame.draw.rect(self.screen, (255, 50, 50), o_rect)
            
        # Draw agent (Blue)
        a_rect = pygame.Rect(self.agent_pos[1] * self.cell_size, self.agent_pos[0] * self.cell_size, self.cell_size, self.cell_size)
        pygame.draw.rect(self.screen, (50, 150, 255), a_rect)
        
        # Draw Stats Text
        mode_text = "[TESTING MODE]" if is_test else f"Ep: {episode} | Eps: {epsilon:.2f}"
        color = (255, 255, 0) if is_test else (255, 255, 255)
        text_surf = self.font.render(f"{mode_text} | Step: {steps}", True, color)
        self.screen.blit(text_surf, (5, 5))
        
        pygame.display.flip()
        
        # Slower visual speed during Testing Mode so we can watch it
        if is_test:
            self.clock.tick(10)
        else:
            self.clock.tick(100) # Fast training

# ==========================================
# 2. THE GENERALIZED Q-LEARNING MODEL
# ==========================================
if __name__ == "__main__":
    env = DynamicObstacleEnv(grid_size=8, num_obstacles=8, render_mode=True)
    
    Q_table = {}
    alpha = 0.1
    gamma = 0.9
    epsilon = 1.0 
    
    print("\n--- TRAINING MODEL ---")
    
    window_wins = 0
    
    for episode in range(1, 1001): # Train for 1000 random-map episodes
        state = env.reset()
        done = False
        steps = 0
        max_steps = 100
        total_reward = 0
        
        while not done and steps < max_steps:
            env.render(episode, steps, epsilon)
            
            if state not in Q_table:
                Q_table[state] = {0:0, 1:0, 2:0, 3:0}
                
            if random.uniform(0, 1) < epsilon:
                action = random.choice([0,1,2,3])
            else:
                action = max(Q_table[state], key=Q_table[state].get)
            
            next_state, reward, done = env.step(action)
            total_reward += reward
            
            if next_state not in Q_table:
                Q_table[next_state] = {0:0, 1:0, 2:0, 3:0}
                
            old_val = Q_table[state][action]
            next_max = max(Q_table[next_state].values())
            Q_table[state][action] = old_val + alpha * (reward + gamma * next_max - old_val)
            
            state = next_state
            steps += 1
            
        # Win Tracking
        if env.agent_pos == env.target_pos:
            window_wins += 1
            
        epsilon = max(0.01, epsilon * 0.995)
        
        if episode % 20 == 0:
            success_rate = (window_wins / 20.0) * 100
            print(f"Episode {episode:4d} | Reward: {total_reward:6.1f} | Eps: {epsilon:.3f} | States: {len(Q_table)} | Success Rate: {success_rate:3.0f}%")
            window_wins = 0

    print(f"\nTraining Complete! Built Knowledge base of {len(Q_table)} relative states.")
    
    # -------------------------
    # 3. DEMO / TESTING MODE
    # -------------------------
    print("\n--- TESTING MODEL (NO EXPLORATION) ---")
    epsilon = 0.0
    
    for test_ep in range(1, 11):
        state = env.reset()
        done = False
        steps = 0
        
        while not done and steps < 100:
            env.render(test_ep, steps, epsilon, is_test=True)
            
            if state not in Q_table:
                action = random.choice([0,1,2,3]) # Fallback if state completely unknown
            else:
                action = max(Q_table[state], key=Q_table[state].get)
                
            next_state, reward, done = env.step(action)
            state = next_state
            steps += 1
            
        status = "REACHED GOAL" if env.agent_pos == env.target_pos else "FAILED"
        print(f"Test Ep {test_ep:2d} | Status: {status} | Steps Taken: {steps}")

    print("\nDemo finished. Close the Pygame window to exit.")
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
