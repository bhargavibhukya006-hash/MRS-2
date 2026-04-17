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
EPISODES = 200
MAX_STEPS = 100
TARGET_UPDATE_INTERVAL = 20

# ==========================================
# REWARD FUNCTION (Per Specification)
# ==========================================
def compute_training_reward(agent_id, world, old_pos, new_pos, action_type, collision_prevented, task_completed):
    reward = 0
    
    # +10 -> task completed
    if task_completed:
        reward += 10
        
    # +2 -> moved closer to target
    target = world.target_position
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
        coord.allocate_tasks() # Ensure roles are assigned for reward calculation if needed
        
        episode_reward = 0
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
            
            # 1. State Capture & Action Selection
            for aid in active_agents:
                # Capture State (using RL module's logic)
                target = world.target_position
                pos = world.agent_positions[aid]
                nearby = [p for i, p in world.agent_positions.items() if i != aid and abs(p[0]-pos[0])+abs(p[1]-pos[1]) <= 3]
                failed = [p for i, p in world.agent_positions.items() if world.agent_status[i] == config.STATUS_BLOCKED]
                states[aid] = q_agent.get_state(pos, target, nearby, 0, failed) # Both agents use same state logic
                
                # Choose Action
                if MODE == "Q":
                    action = q_agent.choose_action(states[aid])
                    actions[aid] = action
                    action_types[aid] = "WAIT" if action in ["WAIT", "REROUTE", "TAKE_OVER"] else "MOVE"
                else: # DQN
                    act_idx = dqn_agent.choose_action(states[aid])
                    actions[aid] = act_idx
                    action_str = dqn_agent.act[act_idx]
                    action_types[aid] = "WAIT" if action_str in ["WAIT", "REROUTE", "TAKE_OVER"] else "MOVE"

                # A* Decides Direction if "MOVE"
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
            world.update_positions(safe_actions)
            
            # 3. Check Success
            success = world.check_joint_task_complete()
            
            # 4. Reward & Update
            for aid in active_agents:
                collision_prevented = (intended_actions[aid] != safe_actions[aid] and intended_actions[aid] != old_positions[aid])
                
                reward = compute_training_reward(
                    aid, world, old_positions[aid], world.agent_positions[aid],
                    action_types[aid], collision_prevented, success
                )
                episode_reward += reward
                
                # Next State
                pos = world.agent_positions[aid]
                nearby = [p for i, p in world.agent_positions.items() if i != aid and abs(p[0]-pos[0])+abs(p[1]-pos[1]) <= 3]
                failed = [p for i, p in world.agent_positions.items() if world.agent_status[i] == config.STATUS_BLOCKED]
                next_state = q_agent.get_state(pos, target, nearby, 0, failed)
                
                # Update
                if MODE == "Q":
                    q_agent.update_q(states[aid], actions[aid], reward, next_state)
                else: # DQN
                    dqn_agent.store(states[aid], actions[aid], reward, next_state, success)
                    dqn_agent.train()
            
            if MODE == "DQN" and (episode * MAX_STEPS + step) % TARGET_UPDATE_INTERVAL == 0:
                dqn_agent.update_target()
                
            if success: break
            
        print(f"Episode {episode+1:3d} | Steps: {steps:3d} | Success: {'YES' if success else ' NO'} | Avg Reward: {episode_reward/len(active_agents) if active_agents else 0:.2f}")

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
