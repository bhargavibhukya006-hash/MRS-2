# main.py

from world import World
from coordination import Coordinator
from pathfinding import astar
from visualization import Visualizer
import config
import math

class RLPolicy:
    """
    Hook for Deep Learning Neural Network Layer.
    Currently maps states to A* directions, but structure supports 
    tensor-based action selection for future training.
    """
    def get_action(self, aid, world, goal):
        path = astar(world.agent_positions[aid], goal, world)
        if len(path) > 1:
            return path[1]
        return world.agent_positions[aid]

def decide_next_move(aid, world, goal, intended_actions, mode="PREDICT", policy=None, use_rl=False):
    if use_rl and policy:
        next_step = policy.get_action(aid, world, goal)
        return next_step, [], False
        
    path = astar(world.agent_positions[aid], goal, world)
    wait_triggered = False
    if len(path) > 1:
        next_step = path[1]
        if mode == "PREDICT" and next_step in intended_actions.values():
            return world.agent_positions[aid], path, True
        return next_step, path, False
    return world.agent_positions[aid], path, False

# ==========================================
# MAIN SIMULATION
# ==========================================

w = World()
coord = Coordinator(w)
vis = Visualizer(w)
rl_agent = RLPolicy()

metrics = {"steps": 0, "collisions": 0, "waits": 0, "task_completed": False}
use_rl = False # Hackathon toggle

initial_dist = 0
for step in range(100):
    if metrics["task_completed"]:
        break

    # 1. Recovery Logic
    coord.update_failures()
    metrics["steps"] += 1
    
    # 2. Allocation
    coord.allocate_tasks()
    
    # Calculate Completion %
    target = w.target_position
    primary_pos = None
    for aid, role in w.agent_roles.items():
        if role == "PRIMARY_CARRIER":
            primary_pos = w.agent_positions[aid]
            break
            
    if primary_pos:
        curr_dist = abs(primary_pos[0] - target[0]) + abs(primary_pos[1] - target[1])
        if step == 0: initial_dist = curr_dist
        comp_pct = max(0, min(100, int((1 - curr_dist / initial_dist) * 100))) if initial_dist > 0 else 100
    else:
        comp_pct = 0

    # 3. Decision
    intended = {}
    paths = {}
    active = w.get_active_agents()
    
    for aid in active:
        role = w.agent_roles[aid]
        if role == "PRIMARY_CARRIER": goal = target
        elif role == "SECONDARY_CARRIER": 
            # Sub-goal: position near target
            goal = (target[0], max(0, target[1]-1))
        else: goal = target
        
        next_step, path, wait = decide_next_move(aid, w, goal, intended, "PREDICT", rl_agent, use_rl)
        intended[aid] = next_step
        paths[aid] = path
        if wait: metrics["waits"] += 1
        
    # 4. Conflict Resolution
    safe = coord.resolve_collisions(intended)
    for aid, pos in intended.items():
        if pos != safe[aid] and pos != w.agent_positions[aid]:
            metrics["collisions"] += 1
            metrics["waits"] += 1
            
    # 5. Execute & Animate
    old_pos = {aid: pos for aid, pos in w.agent_positions.items()}
    w.update_positions(safe)
    
    # Check Completion
    if w.check_joint_task_complete():
        metrics["task_completed"] = True
        comp_pct = 100

    # Visualization
    vis.animate_high_fidelity(
        w, old_pos, w.agent_positions, paths, metrics, comp_pct
    )

print("\nSimulation Finished.")
while True:
    vis.update()