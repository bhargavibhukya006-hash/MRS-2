# train_rl.py
import torch
import pickle
import random
import config
from world import World
from coordination import Coordinator
from pathfinding import astar
import rl.rl_qlearning as q_agent
import rl.rl_dqn as dqn_agent

# ==========================================
# TRAINING CONFIGURATION
# ==========================================
MODE = "Q" # "Q" or "DQN"
EPISODES = 150
MAX_STEPS = 100
TARGET_UPDATE_INTERVAL = 20

# Stability Helpers
ACTION_MAP = {
    "UP": (-1, 0), "DOWN": (1, 0), "LEFT": (0, -1), "RIGHT": (0, 1)
}
reward_history = []

# ==========================================
# REWARD FUNCTION (Per Specification)
# ==========================================
def compute_training_reward(agent_id, world, old_pos, new_pos, action_type, collision_prevented, success, target, invalid_attempt=False):
    reward = 0
    
    # +10 -> task completed (if mission successful)
    if success:
        reward += 10
        
    # +2 -> moved closer to target (current stage start or end)
    old_dist = abs(old_pos[0] - target[0]) + abs(old_pos[1] - target[1])
    new_dist = abs(new_pos[0] - target[0]) + abs(new_pos[1] - target[1])
    if new_dist < old_dist:
        reward += 2
        
    # -1 -> step penalty
    reward -= 1
    
    # -3 -> unnecessary WAIT
    if action_type == "WAIT" and not collision_prevented:
        reward -= 3
        
    # -10 -> collision (prevented by coordinator)
    if collision_prevented:
        reward -= 10

    # Part 6: Penalty for invalid move (obstacle/out of bounds)
    if invalid_attempt:
        reward -= 5
        
    return reward

# ==========================================
# TRAINING PIPELINE
# ==========================================
def run_training():
    print(f"Starting Training Pipeline: {MODE} Mode")
    
    world = World()
    coord = Coordinator(world)
    
    for episode in range(EPISODES):
        world.reset()
        coord.assign_initial_tasks() # Part 2: New Task System API
        
        episode_total_reward = 0
        steps = 0
        success = False
        
        for step in range(MAX_STEPS):
            steps += 1
            active_agents = world.get_active_agents()
            if not active_agents: break
            
            states = {}
            actions = {}
            action_types = {}
            intended_actions = {}
            goals = {}
            invalid_moves = {aid: False for aid in active_agents}
            
            # 1. State Capture & Action Selection
            for aid in active_agents:
                # Part 3: Dynamic Goals based on Task
                task = coord.get_agent_current_task(aid)
                if not task:
                    target = world.target_position # Fallback
                else:
                    target = task.end if task.picked else task.start
                goals[aid] = target

                # Capture State
                pos = world.agent_positions[aid]
                nearby = [p for i, p in world.agent_positions.items() if i != aid and abs(p[0]-pos[0])+abs(p[1]-pos[1]) <= 3]
                failed = [p for i, p in world.agent_positions.items() if world.agent_status[i] == config.STATUS_BLOCKED]
                states[aid] = q_agent.get_state(pos, target, nearby, 0, failed)
                
                # Choose Action
                if random.random() < 0.1:
                    if MODE == "Q":
                        action = random.choice(q_agent.act)
                    else:
                        action = random.randint(0, dqn_agent.act_size - 1)
                else:
                    if MODE == "Q":
                        action = q_agent.choose_action(states[aid])
                    else: # DQN
                        action = dqn_agent.choose_action(states[aid])

                actions[aid] = action
                action_str = action if MODE == "Q" else dqn_agent.act[action]
                
                # Stability Constraints
                if action_str in ACTION_MAP:
                    delta = ACTION_MAP[action_str]
                    target_pos = (pos[0] + delta[0], pos[1] + delta[1])
                    if not world.is_valid_position(target_pos):
                        action_str = "WAIT"
                        invalid_moves[aid] = True

                action_types[aid] = "WAIT" if action_str in ["WAIT", "REROUTE", "TAKE_OVER"] else "MOVE"

                if action_types[aid] == "MOVE":
                    path = astar(pos, target, world)
                    if len(path) > 1:
                        intended_actions[aid] = path[1]
                    else:
                        intended_actions[aid] = pos
                        action_types[aid] = "WAIT"
                else:
                    intended_actions[aid] = pos

            # 2. Coordinate & Update World
            old_positions = {aid: pos for aid, pos in world.agent_positions.items()}
            safe_actions = coord.resolve_collisions(intended_actions)
            
            # Update Task Progress based on safe movement (Sync logic with main.py)
            for aid, s_pos in safe_actions.items():
                task = coord.get_agent_current_task(aid)
                if task:
                    if s_pos == task.start and not task.picked:
                        task.picked = True
                    if s_pos == task.end and task.picked:
                        task.completed = True
                    if task.picked:
                        task.current_location = s_pos

            world.update_positions(safe_actions)
            
            # 3. Check Success
            success = all(t.completed for t in world.tasks)
            
            # 4. Reward & Update
            for aid in active_agents:
                collision_prevented = (intended_actions[aid] != safe_actions[aid] and intended_actions[aid] != old_positions[aid])
                
                reward = compute_training_reward(
                    aid, world, old_positions[aid], world.agent_positions[aid],
                    action_types[aid], collision_prevented, success, goals[aid], invalid_moves[aid]
                )
                episode_total_reward += reward
                
                # Next State
                pos = world.agent_positions[aid]
                nearby = [p for i, p in world.agent_positions.items() if i != aid and abs(p[0]-pos[0])+abs(p[1]-pos[1]) <= 3]
                failed = [p for i, p in world.agent_positions.items() if world.agent_status[i] == config.STATUS_BLOCKED]
                next_target = goals[aid] # Same target for reward sync
                # If just picked up, next target should be end
                task = coord.get_agent_current_task(aid)
                if task and task.picked and not task.completed:
                    next_target = task.end

                next_state = q_agent.get_state(pos, next_target, nearby, 0, failed)
                
                # Update
                if MODE == "Q":
                    q_agent.update_q(states[aid], actions[aid], reward, next_state)
                else: # DQN
                    dqn_agent.store(states[aid], actions[aid], reward, next_state, success)
                    dqn_agent.train()
            
            if MODE == "DQN" and (episode * MAX_STEPS + step) % TARGET_UPDATE_INTERVAL == 0:
                dqn_agent.update_target()
                
            if success: break
            
        reward_history.append(episode_total_reward)
        print(f"Episode {episode+1} | Steps: {steps} | Success: {'YES' if success else 'NO'} | Total Reward: {episode_total_reward:.1f}")

    # ==========================================
    # SAVE MODELS
    # ==========================================
    if MODE == "Q":
        with open("q_table.pkl", "wb") as f:
            pickle.dump(q_agent.Q, f)
        print("Q-table saved to q_table.pkl")
    else:
        torch.save(dqn_agent.model.state_dict(), "dqn_model.pth")
        print("DQN model saved to dqn_model.pth")

if __name__ == "__main__":
    run_training()
