# Deep Q-Network (DQN) Agent
# Uses experience replay and target network
# Designed for multi-agent coordination
# Currently tested on simulated environment (integration pending)
import random
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque

# ACTIONS:
act = ["UP", "DOWN", "LEFT", "RIGHT", "WAIT", "REROUTE", "TAKE_OVER"]
act_size = len(act)
# HYPERPARAMETERS:
gamma = 0.9
epsilon = 1.0
epsilon_decay = 0.995
epsilon_min = 0.05
lr = 0.001
batch_size = 32
# MEMORY:
memory = deque(maxlen=2000)
# MODEL:
class DQN(nn.Module):
    def __init__(self):
        super(DQN, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(6, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, act_size))
    def forward(self, x):
        return self.net(x)
model = DQN()
target_model = DQN()
target_model.load_state_dict(model.state_dict())

optimizer = optim.Adam(model.parameters(), lr=lr)
loss_fn = nn.MSELoss()
# STATE NORMALIZATION:
def state_to_tensor(state):
    state = [s / 10.0 for s in state]
    return torch.tensor(state, dtype=torch.float32).unsqueeze(0)  
# ACTION:
def choose_action(state):
    global epsilon
    if random.random() < epsilon:
        action = random.randint(0, act_size - 1)
    else:
        with torch.no_grad():
            q_vals = model(state_to_tensor(state))
            action = torch.argmax(q_vals).item()

    epsilon = max(epsilon_min, epsilon * epsilon_decay)
    return action
# STORE EXPERIENCE:
def store(state, action, reward, next_state, done):
    memory.append((state, action, reward, next_state, done))

# TRAIN:
def train():
    if len(memory) < batch_size:
        return
    batch = random.sample(memory, batch_size)

    for state, action, reward, next_state, done in batch:
        state_t = state_to_tensor(state)
        next_t = state_to_tensor(next_state)

        q_values = model(state_t)
        next_q = target_model(next_t).detach()

        target = q_values.clone().detach()
        if done:
            target[0][action] = reward
        else:
            target[0][action] = reward + gamma * torch.max(next_q).item()
        loss = loss_fn(q_values, target.detach())
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

# UPDATE TARGET NETWORK:
def update_target():
    target_model.load_state_dict(model.state_dict())

# STATE:
def get_state(agent_pos, target_pos, nearby_agents, cnflt, failed_agents):
    dx = target_pos[0] - agent_pos[0]
    dy = target_pos[1] - agent_pos[1]
    dist = abs(dx) + abs(dy)

    if nearby_agents:
        min_agent_dist = min(
            abs(agent_pos[0] - a[0]) + abs(agent_pos[1] - a[1])
            for a in nearby_agents)
    else:
        min_agent_dist = 10
    return (dx, dy, dist, min_agent_dist, cnflt, len(failed_agents))

# REWARD:
def compute_reward(cnflt, action, progress,task_done, collision,failure_detected, takeover_success,dist_change):
    reward = 0

    if collision:
        reward -= 20

    if cnflt:
        if act[action] == "WAIT":
            reward += 8
        else:
            reward -= 8

    if dist_change < 0:
        reward += 6
    elif dist_change > 0:
        reward -= 4

    if progress:
        reward += 4

    if task_done:
        reward += 15

    if failure_detected:
        if act[action] == "TAKE_OVER":
            reward += 15
        else:
            reward -= 8

    if takeover_success:
        reward += 20
    return reward

# TEST:
if __name__ == "__main__":
    agent_pos = (0, 0)
    target_pos = (5, 5)
    prev_dist = 10

    for step in range(200):
        nearby_agents = [(1, 0), (0, 1)]
        cnflt = random.choice([0, 1])
        collision = False
        progress = random.choice([True, False])
        task_done = random.choice([True, False])

        failure_detected = random.choice([0, 1])
        failed_agents = [(2, 2)] if failure_detected else []
        takeover_success = random.choice([True, False])

        state = get_state(agent_pos, target_pos, nearby_agents, cnflt, failed_agents)
        action = choose_action(state)

        new_dist = random.randint(0, 10)
        dist_change = new_dist - prev_dist
        prev_dist = new_dist

        reward = compute_reward(
            cnflt, action, progress, task_done,
            collision, failure_detected, takeover_success, dist_change)

        next_state = get_state(agent_pos, target_pos, nearby_agents, 0, failed_agents)
        done = task_done or takeover_success
        store(state, action, reward, next_state, done)
        train()
        if step > 0 and step % 20 == 0:
            update_target()
        print(f"Step {step:3d} | Action: {act[action]:10s} | Reward: {reward}")