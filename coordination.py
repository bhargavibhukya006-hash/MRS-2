# coordination.py
# Multi-agent logic (collision, task, failure)

from typing import Dict, Tuple
from world import World
import config

class Coordinator:
    """
    Module 2: Coordination Logic (Collision Avoidance, Task Allocation, Failure Handling)
    Integrates directly with world.py (Central State) and config.py (Constants).
    """

    def __init__(self, world: World):
        # We hold a reference to the centralized world state
        self.world = world

    # ==========================================
    # LOGIC 1: TASK ALLOCATION
    # ==========================================
    def allocate_tasks(self) -> Dict[int, str]:
        """
        Dynamically allocates tasks to bots based on their distance to the target.
        Reads state entirely from self.world.
        Returns the updated roles.
        """
        active_agents = self.world.get_active_agents()
        
        if not active_agents:
            print("COORDINATOR: All agents are BLOCKED. Task failed.")
            return self.world.agent_roles

        # Calculate distances to target
        target = self.world.target_position
        distances = {}
        for aid in active_agents:
            pos = self.world.agent_positions.get(aid)
            if pos:
                dist = abs(pos[0] - target[0]) + abs(pos[1] - target[1])
                distances[aid] = dist

        # Sort active agents by proximity
        sorted_agents = sorted(distances.keys(), key=lambda aid: distances[aid])
        
        # Clear old roles (default to SUPPORT)
        for aid in active_agents:
            self.world.agent_roles[aid] = "SUPPORT"
            
        # Assign roles based on proximity
        if len(sorted_agents) >= 1:
            self.world.agent_roles[sorted_agents[0]] = "PRIMARY_CARRIER"
        if len(sorted_agents) >= 2:
            self.world.agent_roles[sorted_agents[1]] = "SECONDARY_CARRIER"
        if len(sorted_agents) >= 3:
            self.world.agent_roles[sorted_agents[2]] = "SCOUT"
            
        print(f"COORDINATOR: Tasks allocated -> {self.world.agent_roles}")
        return self.world.agent_roles

    # ==========================================
    # LOGIC 2: COLLISION AVOIDANCE
    # ==========================================
    def resolve_collisions(self, intended_actions: Dict[int, Tuple[int, int]]) -> Dict[int, Tuple[int, int]]:
        """
        Filters intended movements to ensure no two agents occupy the same tile, 
        and no two agents cross each other's path.
        
        INPUTS:
            - Module 1/3 (RL/Agents): `intended_actions` mapping agent_id -> (x,y)
        OUTPUTS:
            - Returns `safe_actions` which Environment will safely apply to World.
        """
        safe_actions = {}
        claimed_spots = set()
        
        # 1. Process static/blocked agents to reserve their spots
        for aid in range(self.world.num_agents):
            if self.world.agent_status[aid] == config.STATUS_BLOCKED or aid not in intended_actions:
                safe_pos = self.world.agent_positions[aid]
                safe_actions[aid] = safe_pos
                claimed_spots.add(safe_pos)

        # 2. Priority for movement
        priority_map = {"PRIMARY_CARRIER": 1, "SECONDARY_CARRIER": 2, "SCOUT": 3, "SUPPORT": 4, "UNASSIGNED": 5}
        active_agents = [aid for aid in self.world.get_active_agents() if aid in intended_actions]
        
        # Sort so higher priority resolves movement first
        active_agents.sort(key=lambda aid: priority_map.get(self.world.agent_roles[aid], 99))

        for aid in active_agents:
            curr_pos = self.world.agent_positions[aid]
            intended_pos = intended_actions[aid]
            action_is_safe = True

            # Check A: Target spot claimed?
            if intended_pos in claimed_spots:
                action_is_safe = False
                
            # Check B: Swapping places?
            if action_is_safe:
                for processed_aid, processed_safe_pos in safe_actions.items():
                    if processed_aid != aid:
                        processed_curr_pos = self.world.agent_positions[processed_aid]
                        if processed_safe_pos == curr_pos and processed_curr_pos == intended_pos:
                            action_is_safe = False
                            break

            # If unsafe, force Wait
            if not action_is_safe:
                safe_actions[aid] = curr_pos
            else:
                safe_actions[aid] = intended_pos
                
            claimed_spots.add(safe_actions[aid])
            
        return safe_actions

    # ==========================================
    # LOGIC 3: FAILURE HANDLING
    # ==========================================
    def handle_agent_failure(self, failed_agent_id: int):
        """
        Called when an agent malfunctions. Disables and reallocates task.
        """
        if self.world.agent_status[failed_agent_id] == config.STATUS_BLOCKED:
            return # Already blocked
            
        print(f"COORDINATOR [ALERT]: Agent {failed_agent_id} has suffered a critical failure.")
        
        self.world.agent_status[failed_agent_id] = config.STATUS_BLOCKED
        self.world.agent_roles[failed_agent_id] = "DEAD"
        
        print("COORDINATOR: Initiating emergency task reallocation...")
        self.allocate_tasks()


if __name__ == "__main__":
    # Quick Test Execution
    w = World()
    # Mock data
    w.agent_positions = {0: (2, 2), 1: (3, 3), 2: (8, 8)}
    w.target_position = (10, 10)
    
    coord = Coordinator(w)
    print("\n--- TEST 1: TASK ALLOCATION ---")
    coord.allocate_tasks()
    
    print("\n--- TEST 2: COLLISION AVOIDANCE ---")
    intended_moves = {0: (3, 3), 1: (3, 3), 2: (9, 8)} # Agent 0 and 1 try to go to (3,3)
    safe = coord.resolve_collisions(intended_moves)
    print(f"Intended: {intended_moves}\nSafe Results: {safe}")
    
    print("\n--- TEST 3: FAILURE HANDLING ---")
    coord.handle_agent_failure(failed_agent_id=2)