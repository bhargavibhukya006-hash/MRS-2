# world.py
# Central environment state for multi-agent system

import config

class World:
    def __init__(self):
        # ==========================================
        # BASIC SETTINGS
        # ==========================================
        self.grid_size = 15
        self.num_agents = 3

        # ==========================================
        # AGENT STATE
        # ==========================================
        self.agent_positions = {
            0: (1, 1),
            1: (2, 2),
            2: (3, 3)
        }

        self.agent_roles = {
            0: "UNASSIGNED",
            1: "UNASSIGNED",
            2: "UNASSIGNED"
        }

        self.agent_status = {
            0: config.STATUS_ACTIVE,
            1: config.STATUS_ACTIVE,
            2: config.STATUS_ACTIVE
        }

        # ==========================================
        # TARGET
        # ==========================================
        self.target_position = (12, 12)

        # ==========================================
        # OBSTACLES
        # ==========================================
        self.obstacles = set([
            (5, 5), (5, 6), (5, 7),
            (8, 8), (9, 8)
        ])

    # ==========================================
    # GET ACTIVE AGENTS
    # ==========================================
    def get_active_agents(self):
        return [
            aid for aid in self.agent_status
            if self.agent_status[aid] == config.STATUS_ACTIVE
        ]

    # ==========================================
    # CHECK VALID POSITION
    # ==========================================
    def is_valid_position(self, pos):
        x, y = pos

        # Check grid bounds
        if x < 0 or y < 0 or x >= self.grid_size or y >= self.grid_size:
            return False

        # Check obstacle
        if pos in self.obstacles:
            return False

        return True

    # ==========================================
    # UPDATE POSITIONS (after collision resolution)
    # ==========================================
    def update_positions(self, safe_actions):
        for aid, pos in safe_actions.items():
            if self.is_valid_position(pos):
                self.agent_positions[aid] = pos

    # ==========================================
    # RESET WORLD (optional for testing/RL)
    # ==========================================
    def reset(self):
        self.agent_positions = {
            0: (1, 1),
            1: (2, 2),
            2: (3, 3)
        }

        self.agent_roles = {
            0: "UNASSIGNED",
            1: "UNASSIGNED",
            2: "UNASSIGNED"
        }

        self.agent_status = {
            0: config.STATUS_ACTIVE,
            1: config.STATUS_ACTIVE,
            2: config.STATUS_ACTIVE
        }

        self.target_position = (12, 12)

    # ==========================================
    # PRINT STATE (DEBUGGING)
    # ==========================================
    def print_state(self):
        print("\nWORLD STATE:")
        for aid in range(self.num_agents):
            print(f"Agent {aid}: Pos={self.agent_positions[aid]}, "
                  f"Role={self.agent_roles[aid]}, "
                  f"Status={self.agent_status[aid]}")
        print(f"Target: {self.target_position}")