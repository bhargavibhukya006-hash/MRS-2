# Q-Learning Agent (Tabular)
# Baseline RL approach for comparison with DQN
# Uses epsilon-greedy exploration
# Currently tested on simulated environment (integration pending)
import random

# ACTIONS:
act = ["UP", "DOWN", "LEFT", "RIGHT", "WAIT", "REROUTE", "TAKE_OVER"]
# Q-TABLE:
Q = {}
# HYPERPARAMETERS:
alpha = 0.1
gamma = 0.9
epsilon = 0.2   # will decay

# STATE:
def get_state(agent_pos,target_pos, nearby_agents, cnflt,failed_agents):
    dx = target_pos[0] - agent_pos[0]
    dy = target_pos[1] - agent_pos[1]
    dist = abs(dx) + abs(dy)
    if nearby_agents:
        min_agent_dist = min(
            abs(agent_pos[0] - a[0]) + abs(agent_pos[1] - a[1])
            for a in nearby_agents)
    else:
        min_agent_dist = -1
    return (
        dx, dy,
        dist,
        min_agent_dist,   
        cnflt,
        len(failed_agents))

# ACTION SELECTION:
def choose_action(state):
    global epsilon
    if state not in Q:
        Q[state] = {a: 0 for a in act}
    # Exploration
    if random.uniform(0, 1) < epsilon:
        action = random.choice(act)
    else:
        action = max(Q[state], key=Q[state].get)
    # Epsilon decay
    epsilon = max(0.05, epsilon * 0.995)
    return action


# Q UPDATE:
def update_q(state, action, reward, next_state):
    if next_state not in Q:
        Q[next_state] = {a: 0 for a in act}
    old = Q[state][action]
    next_max = max(Q[next_state].values())
    Q[state][action] = old + alpha * (reward + gamma * next_max - old)


# REWARD (BALANCED):
def compute_reward(cnflt,action, progress,task_done,collision,failure_detected, takeover_success, dist_change):
    reward = 0

    # collision penalty
    if collision:
        reward -= 20

    # conflict handling
    if cnflt:
        if action == "WAIT":
            reward += 8
        else:
            reward -= 8

    # movement toward goal
    if dist_change < 0:
        reward += 6
    elif dist_change > 0:
        reward -= 4

    # general progress
    if progress:
        reward += 4

    # task completion
    if task_done:
        reward += 15

    # failure handling
    if failure_detected:
        if action == "TAKE_OVER":
            reward += 15
        else:
            reward -= 8

    # successful takeover
    if takeover_success:
        reward += 20
    return reward

# TEST:
if __name__ == "__main__":
    agent_pos = (0, 0)
    target_pos = (5, 5)
    prev_dist = abs(target_pos[0] - agent_pos[0]) + abs(target_pos[1] - agent_pos[1])
    for step in range(50):

        # Simulated environment
        nearby_agents = [(1, 0), (0, 1)]
        cnflt = random.choice([0, 1])
        collision = False
        progress = random.choice([True, False])
        task_done = random.choice([True, False])
        failure_detected = random.choice([0, 1])
        failed_agents = [(2, 2)] if failure_detected else []
        takeover_success = random.choice([True, False])

        # State
        state = get_state(agent_pos,target_pos, nearby_agents, cnflt, failed_agents)

        # Action
        action = choose_action(state)

        # Simulate distance change
        new_dist = random.randint(0, 10)
        dist_change = new_dist - prev_dist
        prev_dist = new_dist

        # Reward
        reward = compute_reward(
            cnflt,
            action,
            progress,
            task_done,
            collision,
            failure_detected,
            takeover_success,
            dist_change)
        next_state = get_state(agent_pos,target_pos,nearby_agents, 0, failed_agents)

        # Update
        update_q(state, action, reward, next_state)
        print(f"Step {step} | Action: {action} | Reward: {reward}")