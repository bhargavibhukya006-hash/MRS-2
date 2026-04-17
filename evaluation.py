import random
import pickle
import torch

from world import World
from coordination import Coordinator
from pathfinding import astar

# RL imports
from rl.rl_dqn import model as dqn_model, state_to_tensor
from rl.rl_dqn import choose_action as dqn_choose
from rl.rl_qlearning import Q, choose_action as q_choose

# ==============================
# CONFIG
# ==============================
EPISODES = 30
MAX_STEPS = 100

MODES = ["RULE", "Q", "DQN"]

# ==============================
# LOAD MODELS
# ==============================

# Load Q-table
try:
    with open("q_table.pkl", "rb") as f:
        Q.update(pickle.load(f))
    print("Loaded Q-table")
except:
    print("No Q-table found")

# Load DQN model
try:
    dqn_model.load_state_dict(torch.load("dqn_model.pth"))
    dqn_model.eval()
    print("Loaded DQN model")
except:
    print("No DQN model found")


# ==============================
# ACTION MAPPING
# ==============================

ACTION_MAP = {
    "UP": (0, -1),
    "DOWN": (0, 1),
    "LEFT": (-1, 0),
    "RIGHT": (1, 0),
    "WAIT": (0, 0),
    "REROUTE": (0, 0),
    "TAKE_OVER": (0, 0)
}


def get_next_pos(pos, move):
    return (pos[0] + move[0], pos[1] + move[1])


# ==============================
# DECISION FUNCTION
# ==============================

def decide_action(world, aid, mode):
    curr_pos = world.agent_positions[aid]
    target = world.target_position

    # A* path (baseline)
    path = astar(curr_pos, target, world)

    if not path or len(path) < 2:
        return curr_pos

    next_step = path[1]

    # RULE MODE
    if mode == "RULE":
        return next_step

    # Build RL state
    nearby_agents = [
        world.agent_positions[a]
        for a in world.agent_positions
        if a != aid
    ]

    state = (
        target[0] - curr_pos[0],
        target[1] - curr_pos[1],
        abs(target[0] - curr_pos[0]) + abs(target[1] - curr_pos[1]),
        len(nearby_agents),
        0,
        0
    )

    # ==========================
    # Q-LEARNING
    # ==========================
    if mode == "Q":
        action = q_choose(state)

        if action == "WAIT":
            return curr_pos

        move = ACTION_MAP.get(action, (0, 0))
        return get_next_pos(curr_pos, move)

    # ==========================
    # DQN
    # ==========================
    if mode == "DQN":
        action_idx = dqn_choose(state)
        action = ["UP","DOWN","LEFT","RIGHT","WAIT","REROUTE","TAKE_OVER"][action_idx]

        if action == "WAIT":
            return curr_pos

        move = ACTION_MAP.get(action, (0, 0))
        return get_next_pos(curr_pos, move)

    return curr_pos


# ==============================
# EVALUATION LOOP
# ==============================

def evaluate_mode(mode):
    success_count = 0
    total_steps = 0

    for ep in range(EPISODES):
        world = World()
        world.reset()

        coord = Coordinator(world)

        for step in range(MAX_STEPS):
            coord.allocate_tasks()

            intended_actions = {}

            for aid in world.get_active_agents():
                intended_actions[aid] = decide_action(world, aid, mode)

            # Collision handling
            safe_actions = coord.resolve_collisions(intended_actions)

            world.update_positions(safe_actions)

            # Check success
            if world.check_joint_task_complete():
                success_count += 1
                total_steps += step + 1
                break

        else:
            total_steps += MAX_STEPS

    success_rate = (success_count / EPISODES) * 100
    avg_steps = total_steps / EPISODES

    return success_rate, avg_steps


# ==============================
# MAIN
# ==============================

if __name__ == "__main__":
    print("\n===== EVALUATION START =====\n")

    for mode in MODES:
        print(f"Running Mode: {mode}")
        success, steps = evaluate_mode(mode)

        print(f"{mode} RESULTS:")
        print(f"Success Rate: {success:.2f}%")
        print(f"Avg Steps: {steps:.2f}")
        print("-" * 30)

    print("\n===== DONE =====")