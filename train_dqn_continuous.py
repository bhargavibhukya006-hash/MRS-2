import torch
import torch.nn as nn
import torch.optim as optim
import random
import math
import os
from collections import deque

# 1. Neural Network Structure (MUST perfectly match the ROS 2 node)
class DQNNetwork(nn.Module):
    def __init__(self, state_size=10, action_size=4):
        super(DQNNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, 64)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, action_size)

    def forward(self, x):
        out = self.fc1(x)
        out = self.relu(out)
        out = self.fc2(out)
        out = self.relu(out)
        return self.fc3(out)

# 2. Emulated Continuous ROS Environment
class ContinuousSwarmEnv:
    def __init__(self):
        self.start_x = 100.0
        self.start_y = 100.0
        self.goal_x = 800.0
        self.goal_y = 600.0
        
        self.rx = self.start_x
        self.ry = self.start_y
        self.speed = 4.0
        self.obstacles = []
        self.reset()
        
    def reset(self):
        self.rx = random.uniform(100, 300)
        self.ry = random.uniform(100, 300)
        self.goal_x = random.uniform(500, 900)
        self.goal_y = random.uniform(500, 900)
        
        # Spawn random continuous obstacles
        self.obstacles = []
        for _ in range(15):
            ox = random.uniform(0, 1000)
            oy = random.uniform(0, 1000)
            if math.hypot(ox - self.rx, oy - self.ry) > 50 and math.hypot(ox - self.goal_x, oy - self.goal_y) > 50:
                self.obstacles.append({"x": ox, "y": oy, "r": 20})
        return self.get_state()

    def raycast(self, angle):
        max_dist = 100.0
        for dist in range(1, int(max_dist), 4):
            test_x = self.rx + dist * math.cos(angle)
            test_y = self.ry + dist * math.sin(angle)
            # Check collision with obstacles
            for obs in self.obstacles:
                if math.hypot(test_x - obs["x"], test_y - obs["y"]) < obs["r"]:
                    return float(dist)
        return max_dist

    def get_state(self):
        # Feature 1 & 2: Normalized Goal vector
        dx_norm = (self.goal_x - self.rx) / 1000.0
        dy_norm = (self.goal_y - self.ry) / 1000.0
        
        # Feature 3 - 10: 8-way Lidar sensor (Simulating sensor_msgs/LaserScan bins)
        bins = [0.0] * 8
        angles = [0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi, -3*math.pi/4, -math.pi/2, -math.pi/4]
        for i, ang in enumerate(angles):
            dist = self.raycast(ang)
            if dist < 25.0: # obstacle super close flag
                bins[i] = 1.0
                
        # Must exactly match dqn_local_planner index orders!
        state_list = [dx_norm, dy_norm, bins[0], bins[4], bins[2], bins[6], bins[1], bins[3], bins[5], bins[7]]
        return state_list

    def step(self, action):
        prev_dist = math.hypot(self.goal_x - self.rx, self.goal_y - self.ry)
        
        # 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
        if action == 0: self.rx += self.speed
        elif action == 1: self.rx -= self.speed
        elif action == 2: self.ry -= self.speed
        elif action == 3: self.ry += self.speed
        
        # Collision Check
        for obs in self.obstacles:
            if math.hypot(obs["x"] - self.rx, obs["y"] - self.ry) < obs["r"] + 12.0:
                return self.get_state(), -10.0, True # Crashing is heavily penalized
                
        # Goal Check
        curr_dist = math.hypot(self.goal_x - self.rx, self.goal_y - self.ry)
        if curr_dist < 20.0:
            return self.get_state(), 100.0, True # Big reward
            
        # Distance Reward (Shaping)
        reward = (prev_dist - curr_dist) * 0.1
        
        # Time Penalty
        reward -= 0.05
        
        return self.get_state(), reward, False

# 3. DQN Training Loop
def train():
    env = ContinuousSwarmEnv()
    model = DQNNetwork()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    loss_fn = nn.MSELoss()
    
    memory = deque(maxlen=20000)
    batch_size = 64
    gamma = 0.95
    epsilon = 1.0
    epsilon_decay = 0.995
    epsilon_min = 0.05
    
    episodes = 500
    print(f"Starting Training for Continuous ROS 2 State Space...")
    
    for ep in range(episodes):
        state = env.reset()
        total_reward = 0
        
        for step in range(500): # max steps
            # Epsilon-Greedy Action
            if random.random() < epsilon:
                action = random.randint(0,3)
            else:
                state_tensor = torch.FloatTensor(state).unsqueeze(0)
                with torch.no_grad():
                    q_values = model(state_tensor)
                action = torch.argmax(q_values).item()
                
            next_state, reward, done = env.step(action)
            memory.append((state, action, reward, next_state, done))
            state = next_state
            total_reward += reward
            
            # Replay Memory Training
            if len(memory) > batch_size:
                batch = random.sample(memory, batch_size)
                
                states = torch.FloatTensor([b[0] for b in batch])
                actions = torch.LongTensor([b[1] for b in batch])
                rewards = torch.FloatTensor([b[2] for b in batch])
                next_states = torch.FloatTensor([b[3] for b in batch])
                dones = torch.FloatTensor([b[4] for b in batch])
                
                # Q-learning exact logic
                curr_q = model(states).gather(1, actions.unsqueeze(1)).squeeze(1)
                with torch.no_grad():
                    max_next_q = model(next_states).max(1)[0]
                    target_q = rewards + gamma * max_next_q * (1 - dones)
                    
                loss = loss_fn(curr_q, target_q)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                
            if done:
                break
                
        epsilon = max(epsilon_min, epsilon * epsilon_decay)
        
        if ep % 10 == 0:
            print(f"Episode {ep:03d} | Reward: {total_reward:6.2f} | Epsilon: {epsilon:.2f} | Loss: {loss.item() if 'loss' in locals() else 0.0:.4f}")

    # Save exactly to the path expected by the ROS 2 node
    save_path = os.path.expanduser('~/MRS-2/dqn_model.pth')
    torch.save(model.state_dict(), save_path)
    print(f"\n✅ Training Complete. Model saved over to {save_path}!")

if __name__ == '__main__':
    train()
