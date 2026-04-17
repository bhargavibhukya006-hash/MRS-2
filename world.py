# world.py
# Central environment state for multi-agent system

import random
import config

class Task:
    def __init__(self, name, start, end):
        self.name = name
        self.start = start
        self.end = end
        self.picked = False
        self.completed = False
        self.failed = False
        self.current_location = start

class World:
    def __init__(self):
        # 4. USE config constants:
        self.grid_size = config.GRID_SIZE
        self.num_agents = config.NUM_AGENTS
        self.num_obstacles = config.NUM_OBSTACLES

        # Core state
        self.agent_positions = {}
        self.agent_status = {}
        self.agent_roles = {}

        self.target_position = None
        # 3. ADD obstacles as: self.obstacles = set()
        self.obstacles = set()
        self.tasks = [] # Restoration of tasks list

        # 5. Call self.reset() in __init__
        self.reset()

    # ==========================================
    # 1. ADD a reset() function
    # ==========================================
    def reset(self):
        self.agent_positions = {}
        self.agent_status = {}
        self.agent_roles = {}
        self.obstacles = set()
        self.tasks = []

        occupied = set()

        # Helper to get a random position that is not occupied
        def get_random_pos():
            while True:
                pos = (
                    random.randint(0, self.grid_size - 1),
                    random.randint(0, self.grid_size - 1)
                )
                # 2. SAFE SPAWN RULES: Ensure NO overlap
                if pos not in occupied:
                    return pos

        # --------- TARGET SPAWN ----------
        self.target_position = get_random_pos()
        occupied.add(self.target_position)

        # --------- OBSTACLE SPAWN ----------
        for _ in range(self.num_obstacles):
            obs_pos = get_random_pos()
            self.obstacles.add(obs_pos)
            occupied.add(obs_pos)

        # --------- AGENT SPAWN ----------
        for aid in range(self.num_agents):
            self.agent_positions[aid] = get_random_pos()
            occupied.add(self.agent_positions[aid])
            
            self.agent_status[aid] = config.STATUS_ACTIVE
            self.agent_roles[aid] = "UNASSIGNED"

        # --------- TASK SPAWN (Restored Compatibility) ----------
        # PART 1 FIX: task.start and task.end MUST be added to `occupied`
        # so they cannot spawn on obstacle cells — this was the root cause
        # of agents appearing to pass through obstacles (unreachable goals).
        for i in range(1, 4):
            start = get_random_pos()
            occupied.add(start)          # reserve start cell
            end = get_random_pos()
            occupied.add(end)            # reserve end cell
            self.tasks.append(Task(f"T{i}", start, end))

        # PART 3: Validate no task endpoint landed in obstacles (safety assertion)
        for t in self.tasks:
            assert t.start not in self.obstacles, f"SPAWN ERROR: Task {t.name} start {t.start} is inside an obstacle!"
            assert t.end not in self.obstacles,   f"SPAWN ERROR: Task {t.name} end {t.end} is inside an obstacle!"
        for aid, pos in self.agent_positions.items():
            assert pos not in self.obstacles, f"SPAWN ERROR: Agent {aid} spawned inside obstacle at {pos}!"

        # DEBUG SUPPORT
        print("\n--- RESET COMPLETE ---")
        print(f"Obstacles in world: {len(self.obstacles)} → {sorted(self.obstacles)}")
        print(f"Agent Positions: {self.agent_positions}")
        print(f"Target Position: {self.target_position}")
        print(f"Tasks: {[(t.name, t.start, t.end) for t in self.tasks]}")

    # ==========================================
    # 7. DO NOT REMOVE: get_active_agents()
    # ==========================================
    def get_active_agents(self):
        return [
            aid for aid in range(self.num_agents)
            if self.agent_status.get(aid) == config.STATUS_ACTIVE
        ]

    # ==========================================
    # 7. DO NOT REMOVE: update_positions()
    # ==========================================
    def update_positions(self, new_positions):
        for aid, pos in new_positions.items():
            if self.agent_status.get(aid) == config.STATUS_ACTIVE:
                self.agent_positions[aid] = pos

    # ==========================================
    # 6. ADD function: is_valid(self, pos)
    # ==========================================
    def is_valid(self, pos):
        x, y = pos

        # Return False if out of bounds
        if x < 0 or x >= self.grid_size or y < 0 or y >= self.grid_size:
            return False

        # Return False if position is obstacle
        if pos in self.obstacles:
            return False

        return True

    def is_valid_position(self, pos):
        """Compatibility alias for is_valid"""
        return self.is_valid(pos)

    # ==========================================
    # Legacy logic kept for architecture consistency
    # ==========================================
    def check_joint_task_complete(self):
        primary = None
        secondary = None

        for aid, role in self.agent_roles.items():
            if role == "PRIMARY_CARRIER":
                primary = self.agent_positions.get(aid)
            elif role == "SECONDARY_CARRIER":
                secondary = self.agent_positions.get(aid)

        if not primary or not secondary:
            return False

        if primary != self.target_position:
            return False

        dist = abs(secondary[0] - self.target_position[0]) + abs(secondary[1] - self.target_position[1])
        return dist <= 1

    def print_state(self):
        print("\nWORLD STATE:")
        for aid in range(self.num_agents):
            print(
                f"Agent {aid}: Pos={self.agent_positions.get(aid)}, "
                f"Role={self.agent_roles.get(aid)}, "
                f"Status={self.agent_status.get(aid)}"
            )
        print(f"Target: {self.target_position}")