# main.py

from world import World
from coordination import Coordinator
from pathfinding import astar
from visualization import Visualizer
import config
import math
import random
import pickle
import torch

# PART 1: Import RL Modules
import rl.rl_qlearning as q_agent
import rl.rl_dqn as dqn_agent

# PART 2: Mode Switching
MODE = "RULE"   # Options: "RULE", "Q", "DQN"

# PART 3: Action Mapping
ACTION_MAP = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
    "WAIT": (0, 0),
    "REROUTE": (0, 0),
    "TAKE_OVER": (0, 0)
}
ACT_LIST = ["UP", "DOWN", "LEFT", "RIGHT", "WAIT", "REROUTE", "TAKE_OVER"]

# Evaluation Parameters
EPI_COUNT = 5
MAX_STEPS = 100

# PART 1 (Continued): Load Models
if MODE == "Q":
    try:
        with open("q_table.pkl", "rb") as f:
            q_agent.Q = pickle.load(f)
        q_agent.epsilon = 0 # No exploration during inference
        print("SYSTEM: Pre-trained Q-table loaded successfully.")
    except FileNotFoundError:
        print("WARNING: q_table.pkl not found. Agent will act randomly.")

elif MODE == "DQN":
    try:
        dqn_agent.model.load_state_dict(torch.load("dqn_model.pth"))
        dqn_agent.model.eval()
        dqn_agent.epsilon = 0 # No exploration during inference
        print("SYSTEM: Pre-trained DQN weights loaded successfully.")
    except FileNotFoundError:
        print("WARNING: dqn_model.pth not found. Agent will act randomly.")

class RLPolicy:
    def get_action(self, aid, world, goal):
        path = astar(world.agent_positions[aid], goal, world)
        if len(path) > 1:
            return path[1]
        return world.agent_positions[aid]

def decide_next_move(aid, world, goal, intended_actions, mode="PREDICT", policy=None, use_rl=False):
    # Rule-based fallback or pure rule mode
    path = astar(world.agent_positions[aid], goal, world)
    wait_triggered = False
    
    if mode == "RULE":
        if len(path) > 1:
            next_step = path[1]
            if next_step in intended_actions.values():
                return world.agent_positions[aid], path, True
            return next_step, path, False
        return world.agent_positions[aid], path, False

    # RL Logic
    # (To be called inside the episode loop for better state/reward tracking)
    return world.agent_positions[aid], path, False

# ==========================================
# MAIN SIMULATION
# ==========================================

all_metrics = []

# Helper to fetch state for agents
def get_current_state(aid, world, target, cnflt=0):
    pos = world.agent_positions[aid]
    nearby = [p for i, p in world.agent_positions.items() if i != aid and abs(p[0]-pos[0])+abs(p[1]-pos[1]) <= 3]
    failed = [p for i, p in world.agent_positions.items() if world.agent_status[i] == config.STATUS_BLOCKED]
    return q_agent.get_state(pos, target, nearby, cnflt, failed)

# Initializer for Visualization (needed for static context)
vis = Visualizer(World())

# Main Episode Loop
for episode in range(EPI_COUNT):
    w = World()
    coord = Coordinator(w)
    
    metrics = {"steps": 0, "collisions": 0, "waits": 0, "task_completed": False}
    initial_dist = 0
    
    print(f"\n===== EPISODE {episode+1}/{EPI_COUNT} | MODE: {MODE} =====")

    for step in range(MAX_STEPS):
        if metrics["task_completed"]:
            break

        # 1. Failure Recovery
        coord.update_failures()
        metrics["steps"] += 1
        
        # 2. Planning
        coord.allocate_tasks()
        target = w.target_position
        
        # Capture initial dist for metrics %
        if step == 0:
            p_pos = next((w.agent_positions[i] for i, r in w.agent_roles.items() if r == "PRIMARY_CARRIER"), None)
            if p_pos: initial_dist = abs(p_pos[0]-target[0]) + abs(p_pos[1]-target[1])
        
        # 3. Decision & State Capture
        # 3. Decision & State Capture
        intended = {}
        paths = {}
        states = {}
        actions = {}
        
        active = w.get_active_agents()
        for aid in active:
            # a. Capture State
            states[aid] = get_current_state(aid, w, target)
            curr = w.agent_positions[aid]
            
            # b. Choose Action
            if MODE == "RULE":
                next_pos, path, wait = decide_next_move(aid, w, target, intended, "RULE")
                intended[aid] = next_pos
                paths[aid] = path
                actions[aid] = "WAIT" if wait or next_pos == curr else "MOVE"
            else:
                # RL Inference (Pre-trained)
                if MODE == "Q":
                    action_str = q_agent.choose_action(states[aid])
                else: # DQN
                    idx = dqn_agent.choose_action(states[aid])
                    action_str = dqn_agent.act[idx]
                
                actions[aid] = action_str
                
                # Hybrid Logic: 75% A* / 25% Direct RL
                if action_str in ["WAIT", "REROUTE", "TAKE_OVER"]:
                    next_pos = curr
                else:
                    roll = random.random()
                    if roll < 0.25: # Direct RL Move
                        delta = ACTION_MAP.get(action_str, (0, 0))
                        next_pos = (curr[0] + delta[0], curr[1] + delta[1])
                    else: # A* Pathfinding Step (Hybrid)
                        path = astar(curr, target, w)
                        next_pos = path[1] if len(path) > 1 else curr
                        paths[aid] = path
                
                # Boundary/Obstacle fallback
                if not w.is_valid_position(next_pos):
                    next_pos = curr
                    
                intended[aid] = next_pos

        # 4. Conflict Resolution (Coordinator)
        safe = coord.resolve_collisions(intended)
        
        # 5. Execute & Learn
        old_positions = {aid: pos for aid, pos in w.agent_positions.items()}
        w.update_positions(safe)
        
        # Check Task
        if w.check_joint_task_complete():
            metrics["task_completed"] = True

        # 6. Visualization
        comp_pct = 0
        p_pos = next((w.agent_positions[i] for i, r in w.agent_roles.items() if r == "PRIMARY_CARRIER"), None)
        if p_pos and initial_dist > 0:
            curr_d = abs(p_pos[0]-target[0]) + abs(p_pos[1]-target[1])
            comp_pct = max(0, min(100, int((1 - curr_d/initial_dist)*100)))
        
        vis.animate_high_fidelity(w, old_positions, w.agent_positions, paths, metrics, comp_pct)

    all_metrics.append(metrics)

# Final comparison
print(f"\n===== SIMULATION COMPLETE | MODE: {MODE} =====")
success_count = sum(1 for m in all_metrics if m["task_completed"])
success_rate = (success_count / len(all_metrics)) * 100 if all_metrics else 0
avg_steps = sum(m["steps"] for m in all_metrics) / len(all_metrics) if all_metrics else 0

print(f"Final Success Rate: {success_rate:.1f}%")
print(f"Avg Steps per Episode: {avg_steps:.1f}")

while True:
    vis.update()