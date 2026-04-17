# main.py

from world import World
from coordination import Coordinator
from pathfinding import astar
from visualization import Visualizer
import random

EPI_COUNT = 3
MAX_STEPS = 200

vis = None

def get_dynamic_obstacles(w, current_aid):
    obs = set(w.obstacles)
    for aid, pos in w.agent_positions.items():
        if aid != current_aid:
            obs.add(pos)
    return obs

for episode in range(EPI_COUNT):
    w = World()

    if vis is None:
        vis = Visualizer(w)
    else:
        vis.world = w

    coord = Coordinator(w)
    coord.assign_initial_tasks()

    metrics = {
        "steps": 0,
        "waits": 0,
        "collisions": 0,   # ✅ REQUIRED
        "task_completed": False
    }
    print(f"\n===== EPISODE {episode+1} =====")

    for step in range(MAX_STEPS):

        # OPTIONAL: comment this if you don't want failure
        if step == 10:
            coord.handle_agent_failure(1)

        if all(t.completed for t in w.tasks):
            print("MISSION COMPLETE")
            metrics["task_completed"] = True
            break

        metrics["steps"] += 1

        old_positions = dict(w.agent_positions)

        intended = {}
        paths = {}

        for aid in w.get_active_agents():

            curr = w.agent_positions[aid]
            task = coord.get_agent_current_task(aid)

            if not task:
                intended[aid] = curr
                continue

            # --- GOAL LOGIC ---
            if not task.picked:
                goal = task.start
            else:
                goal = task.current_location if task.failed else task.end

            # --- DYNAMIC OBSTACLES (KEY FIX) ---
            dyn_obs = get_dynamic_obstacles(w, aid)

            path = astar(curr, goal, w.grid_size, dyn_obs)

            if len(path) > 1:
                next_pos = path[1]
            else:
                next_pos = curr

            intended[aid] = next_pos
            paths[aid] = path

            print(f"Agent {aid}: {curr} → {next_pos} | Goal: {goal}")

        # --- COLLISION RESOLUTION ---
        safe,collisions = coord.resolve_collisions(intended)
        metrics["collisions"] += collisions
        # --- TASK EXECUTION ---
        for aid, pos in safe.items():
            task = coord.get_agent_current_task(aid)
            if not task:
                continue

            if pos == task.start and not task.picked:
                task.picked = True
                task.current_location = pos
                print(f"PICKED: Agent {aid} picked {task.name}")

            if pos == task.end and task.picked:
                task.completed = True
                print(f"DELIVERED: Agent {aid} completed {task.name}")

            if task.picked and not task.completed:
                task.current_location = pos

        w.update_positions(safe)

        # --- VISUAL ---
        done = sum(1 for t in w.tasks if t.completed)
        pct = int((done / len(w.tasks)) * 100)

        vis.animate_high_fidelity(
            w,
            old_positions,
            w.agent_positions,
            paths,
            metrics,
            pct
        )

print("\n===== DONE =====")

while True:
    vis.update()