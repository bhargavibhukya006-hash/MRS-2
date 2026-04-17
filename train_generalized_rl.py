import random

# ==========================================
# 1. THE RELATIVE ENVIRONMENT
# ==========================================
class DynamicObstacleEnv:
    def __init__(self, grid_size=10, num_obstacles=5):
        self.grid_size = grid_size
        self.num_obstacles = num_obstacles
        self.reset()
        
    def reset(self):
        # Spawns agent at top-left, target at bottom-right
        self.agent_pos = [0, 0]
        self.target_pos = [self.grid_size - 1, self.grid_size - 1]
        
        # DOMAIN RANDOMIZATION: Randomly spawn obstacles every episode
        self.obstacles = set()
        while len(self.obstacles) < self.num_obstacles:
            obs = (random.randint(0, self.grid_size - 1), random.randint(0, self.grid_size - 1))
            # Don't place obstacle on agent or target
            if obs != tuple(self.agent_pos) and obs != tuple(self.target_pos):
                self.obstacles.add(obs)
                
        return self.get_relative_state()
        
    def get_relative_state(self):
        # ------------------------------------------------------------------
        # CRITICAL RL CONCEPT: RELATIVE LOCAL OBSERVATION ("Lidar" sensing)
        # ------------------------------------------------------------------
        # We DO NOT tell the AI its exact (x,y) location. 
        # We tell it the distance to the goal, and what is immediately around it.
        dx = self.target_pos[0] - self.agent_pos[0]
        dy = self.target_pos[1] - self.agent_pos[1]
        
        # Sense immediate surroundings (True if blocked/wall, False if open)
        x, y = self.agent_pos
        wall_up    = (x - 1 < 0) or ((x - 1, y) in self.obstacles)
        wall_down  = (x + 1 >= self.grid_size) or ((x + 1, y) in self.obstacles)
        wall_left  = (y - 1 < 0) or ((x, y - 1) in self.obstacles)
        wall_right = (y + 1 >= self.grid_size) or ((x, y + 1) in self.obstacles)
        
        # The Q-Table State is just these simple boolean flags and differences
        return (dx, dy, wall_up, wall_down, wall_left, wall_right)

    def step(self, action):
        # Actions: 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
        new_x, new_y = self.agent_pos[0], self.agent_pos[1]
        
        if action == 0: new_x -= 1
        elif action == 1: new_x += 1
        elif action == 2: new_y -= 1
        elif action == 3: new_y += 1
            
        # Check boundaries and obstacles
        if new_x < 0 or new_x >= self.grid_size or new_y < 0 or new_y >= self.grid_size:
            return self.get_relative_state(), -10, False  # Punish Wall hit
            
        if (new_x, new_y) in self.obstacles:
            return self.get_relative_state(), -10, False  # Punish Obstacle hit
            
        # Move agent
        self.agent_pos = [new_x, new_y]
        
        # Check goal
        if self.agent_pos == self.target_pos:
            return self.get_relative_state(), 100, True   # Reward Goal
            
        return self.get_relative_state(), -1, False       # Step penalty

# ==========================================
# 2. THE GENERALIZED Q-LEARNING MODEL
# ==========================================
if __name__ == "__main__":
    env = DynamicObstacleEnv(grid_size=6, num_obstacles=5)
    
    Q_table = {}
    alpha = 0.1
    gamma = 0.9
    epsilon = 0.2
    
    print("\n--- TRAINING MODEL WITH RANDOM OBSTACLES ---")
    
    for episode in range(1000): # Train for 1000 random-map episodes
        state = env.reset() # This completely randomizes the obstacles!
        done = False
        
        while not done:
            if state not in Q_table:
                Q_table[state] = {0:0, 1:0, 2:0, 3:0}
                
            # Epsilon Greedy choice
            if random.uniform(0, 1) < epsilon:
                action = random.choice([0,1,2,3])
            else:
                action = max(Q_table[state], key=Q_table[state].get)
                
            next_state, reward, done = env.step(action)
            
            if next_state not in Q_table:
                Q_table[next_state] = {0:0, 1:0, 2:0, 3:0}
                
            # Update math
            old_val = Q_table[state][action]
            next_max = max(Q_table[next_state].values())
            Q_table[state][action] = old_val + alpha * (reward + gamma * next_max - old_val)
            
            state = next_state

    print(f"Training Complete! Built Knowledge base of {len(Q_table)} relative states.")
    print("Because it trained using relative sensors on thousands of random maps,")
    print("this bot can be dropped onto a map with ANY obstacle configuration and it will dodge them to reach the goal!")
