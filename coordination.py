# coordination.py

from typing import Dict, Tuple
from world import World
import config

class Coordinator:

    def __init__(self, world: World):
        self.world = world
        self.agent_task_map = {}

    # ----------------------------
    # TASK ASSIGNMENT
    # ----------------------------
    def assign_initial_tasks(self):
        for i in range(min(len(self.world.tasks), self.world.num_agents)):
            self.agent_task_map[i] = [self.world.tasks[i]]
            print(f"ASSIGN: Agent {i} → {self.world.tasks[i].name}")

    def get_agent_current_task(self, aid):
        if aid in self.agent_task_map:
            for task in self.agent_task_map[aid]:
                if not task.completed:
                    return task
        return None

    # ----------------------------
    # COLLISION AVOIDANCE (FIXED)
    # ----------------------------
    def resolve_collisions(self, intended: Dict[int, Tuple[int, int]]):

        safe = {}
        claimed = set()
        collision_count = 0

        # Blocked agents act like obstacles
        blocked_positions = {
            self.world.agent_positions[aid]
            for aid in range(self.world.num_agents)
            if self.world.agent_status[aid] == config.STATUS_BLOCKED
        }

        for aid in sorted(intended.keys()):

            curr = self.world.agent_positions[aid]
            nxt = intended[aid]

            moved = False  # ✅ FIX: always initialized

            # Check if move is unsafe
            if (
                nxt in claimed or
                nxt in blocked_positions or
                nxt in self.world.obstacles or
                not (0 <= nxt[0] < self.world.grid_size and 0 <= nxt[1] < self.world.grid_size)
            ):

                # Try alternative moves (DEADLOCK BREAK)
                alternatives = [
                    (curr[0]+1, curr[1]),
                    (curr[0]-1, curr[1]),
                    (curr[0], curr[1]+1),
                    (curr[0], curr[1]-1),
                ]

                for alt in alternatives:
                    if (
                        0 <= alt[0] < self.world.grid_size and
                        0 <= alt[1] < self.world.grid_size and
                        alt not in self.world.obstacles and
                        alt not in claimed and
                        alt not in blocked_positions
                    ):
                        safe[aid] = alt
                        claimed.add(alt)
                        moved = True
                        break

                # If no alternative found → WAIT
                if not moved:
                    safe[aid] = curr
                    claimed.add(curr)
                    collision_count += 1

            else:
                # Safe move
                safe[aid] = nxt
                claimed.add(nxt)

        print(f"SAFE: {safe}")
        return safe, collision_count

    # ----------------------------
    # FAILURE HANDLING (SMART)
    # ----------------------------
    def handle_agent_failure(self, failed_id):

        if self.world.agent_status[failed_id] == config.STATUS_BLOCKED:
            return

        print(f"FAILURE: Agent {failed_id} blocked")

        self.world.agent_status[failed_id] = config.STATUS_BLOCKED

        task = self.get_agent_current_task(failed_id)
        if not task:
            return

        task.failed = True

        active_agents = self.world.get_active_agents()
        if not active_agents:
            return

        def manhattan(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        target = task.current_location if task.picked else task.start

        # Choose closest agent
        new_agent = min(
            active_agents,
            key=lambda aid: manhattan(self.world.agent_positions[aid], target)
        )

        if new_agent not in self.agent_task_map:
            self.agent_task_map[new_agent] = []

        # Put task at front (priority)
        self.agent_task_map[new_agent].insert(0, task)

        task.failed = False

        print(f"RECOVERY: Task {task.name} → Agent {new_agent}")

        if task.picked:
            print(f"Agent {new_agent} will go to {task.current_location} then {task.end}")
        else:
            print(f"Agent {new_agent} will go to {task.start}")