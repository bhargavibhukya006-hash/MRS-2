# main.py

from world import World
from coordination import Coordinator
from pathfinding import astar
from visualization import Visualizer
import copy

# ==========================================
# DECISION LAYER
# ==========================================
def decide_next_move(aid, start, goal, world, intended_actions, mode="PREDICT"):
    """
    Returns (next_step, path, wait_triggered) using A*.
    In PREDICT mode: avoid stepping into cells already intended by others.
    """
    path = astar(start, goal, world)
    wait_triggered = False

    if len(path) > 1:
        next_step = path[1]
        if mode == "PREDICT" and next_step in intended_actions.values():
            wait_triggered = True
            return start, path, wait_triggered
        return next_step, path, wait_triggered
    return start, path, wait_triggered


# ==========================================
# MAIN SIMULATION
# ==========================================

# Initialize two identical worlds
w_rule = World()
w_pred = copy.deepcopy(w_rule)

coord_rule = Coordinator(w_rule)
coord_pred = Coordinator(w_pred)

# Single visualizer (uses w_rule for grid/obstacles context)
vis = Visualizer(w_rule)

metrics_rule = {"steps": 0, "collisions": 0, "waits": 0}
metrics_pred = {"steps": 0, "collisions": 0, "waits": 0}

# Run simulation
for step in range(25):
    print(f"\n--- STEP {step} ---")
    metrics_rule["steps"] = step + 1
    metrics_pred["steps"] = step + 1

    # 1. Allocate tasks for both
    coord_rule.allocate_tasks()
    coord_pred.allocate_tasks()

    # 2. Simulate failure at step 5
    failed_aid = None
    if step == 5:
        failed_aid = 1
        coord_rule.handle_agent_failure(failed_aid)
        coord_pred.handle_agent_failure(failed_aid)

    # 3. Decision Logic & Intended Actions
    intended_rule = {}
    paths_rule = {}
    
    intended_pred = {}
    paths_pred = {}

    # Process RULE agents
    for aid in w_rule.get_active_agents():
        start = w_rule.agent_positions[aid]
        role = w_rule.agent_roles[aid]
        # Same role-based goal logic for both
        if role == "PRIMARY_CARRIER": goal = w_rule.target_position
        elif role == "SECONDARY_CARRIER": goal = (max(0, w_rule.target_position[0]-1), w_rule.target_position[1])
        elif role == "SCOUT": goal = (w_rule.target_position[0], max(0, w_rule.target_position[1]-2))
        else: goal = w_rule.target_position

        next_step, path, _ = decide_next_move(aid, start, goal, w_rule, intended_rule, mode="RULE")
        intended_rule[aid] = next_step
        paths_rule[aid] = path

    # Process PREDICT agents
    for aid in w_pred.get_active_agents():
        start = w_pred.agent_positions[aid]
        role = w_pred.agent_roles[aid]
        if role == "PRIMARY_CARRIER": goal = w_pred.target_position
        elif role == "SECONDARY_CARRIER": goal = (max(0, w_pred.target_position[0]-1), w_pred.target_position[1])
        elif role == "SCOUT": goal = (w_pred.target_position[0], max(0, w_pred.target_position[1]-2))
        else: goal = w_pred.target_position

        next_step, path, wait_triggered = decide_next_move(aid, start, goal, w_pred, intended_pred, mode="PREDICT")
        intended_pred[aid] = next_step
        paths_pred[aid] = path
        if wait_triggered: metrics_pred["waits"] += 1

    # 4. Resolve Collisions (Track avoided collisions)
    # We compare intended vs safe to count coordinator interventions
    safe_rule = coord_rule.resolve_collisions(intended_rule)
    for aid, pos in intended_rule.items():
        if pos != safe_rule[aid] and pos != w_rule.agent_positions[aid]:
            metrics_rule["collisions"] += 1
            metrics_rule["waits"] += 1

    safe_pred = coord_pred.resolve_collisions(intended_pred)
    for aid, pos in intended_pred.items():
        # Avoid double counting "waits" already counted in decision layer
        if pos != safe_pred[aid] and pos != w_pred.agent_positions[aid]:
            metrics_pred["collisions"] += 1
            metrics_pred["waits"] += 1

    # 5. Update positions
    old_rule = {aid: pos for aid, pos in w_rule.agent_positions.items()}
    w_rule.update_positions(safe_rule)

    old_pred = {aid: pos for aid, pos in w_pred.agent_positions.items()}
    w_pred.update_positions(safe_pred)

    # 6. Calculate Improvement (%)
    # improvement = ((rule_wait - predict_wait) / rule_wait) * 100
    improvement = 0
    if metrics_rule["waits"] > 0:
        improvement = ((metrics_rule["waits"] - metrics_pred["waits"]) / metrics_rule["waits"]) * 100

    # 7. Update Split-Screen Visualization
    vis.animate_two_worlds(
        w_rule, old_rule, w_rule.agent_positions, paths_rule, metrics_rule,
        w_pred, old_pred, w_pred.agent_positions, paths_pred, metrics_pred,
        improvement, failed_aid
    )

print("\nSimulation finished. Close window to exit.")
while True:
    vis.update()