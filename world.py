# world.py
# Central environment state for multi-agent system

import config

class World:
    def __init__(self):
        # ==========================================
        # BASIC SETTINGS
        # ==========================================
        self.grid_size = 30
        self.num_agents = 3

        # ==========================================
        # AGENT STATE
        # ==========================================
        self.agent_positions = {
            0: (2, 2),
            1: (4, 4),
            2: (6, 6)
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
        self.target_position = (24, 24)

        # ==========================================
        # OBSTACLES
        # ==========================================
        self.obstacles = set([
            (10, 10), (10, 11), (10, 12), (10, 13), (10, 14),
            (11, 10), (12, 10),
            (20, 15), (20, 16), (20, 17),
            (15, 20), (16, 20), (17, 20)
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
            0: (2, 2),
            1: (4, 4),
            2: (6, 6)
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

        self.target_position = (24, 24)

    # ==========================================
    # LOGIC: CHECK JOINT TASK COMPLETION
    # ==========================================
    def check_joint_task_complete(self):
        """
        Returns True if:
        - PRIMARY_CARRIER is exactly at target
        - SECONDARY_CARRIER is within 1 cell of target (Manhattan dist <= 1)
        """
        primary_aid = None
        secondary_aid = None
        
        for aid, role in self.agent_roles.items():
            if role == "PRIMARY_CARRIER":
                primary_aid = aid
            elif role == "SECONDARY_CARRIER":
                secondary_aid = aid
                
        if primary_aid is None or secondary_aid is None:
            return False
            
        p_pos = self.agent_positions[primary_aid]
        s_pos = self.agent_positions[secondary_aid]
        target = self.target_position
        
        # PRIMARY must be at target
        if p_pos != target:
            return False
            
        # SECONDARY must be within 1 cell
        dist = abs(s_pos[0] - target[0]) + abs(s_pos[1] - target[1])
        if dist > 1:
            return False
            
        return True

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