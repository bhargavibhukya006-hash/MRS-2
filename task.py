import random

class MultiBotSystem:
    def __init__(self, num_bots=3, grid_size=15):
        self.grid_size = grid_size
        self.num_bots = num_bots
        
        # ================================
        # PERFORMANCE TRACKING
        # ================================
        self.metrics = {
            "tasks_completed": 0,
            "collisions_avoided": 0,
            "reassignments_made": 0,
            "steps_taken": 0
        }
        
        # Bot State Dictionary
        # Now holds a LIST of tasks to track "workload"
        self.bots = {
            i: {
                "pos": self.random_pos(), 
                "status": "ACTIVE", 
                "tasks": [] 
            } for i in range(num_bots)
        }
        
        self.obstacles = set()
        self.randomize_obstacles(num_obstacles=15)
        
        # ================================
        # ADVANCED TASKS
        # Now represented as dictionaries with weights & targets
        # ================================
        self.pending_tasks = [
            {"id": "T1", "type": "MAJOR", "pos": (12, 12)},
            {"id": "T2", "type": "MAJOR", "pos": (13, 10)},
            {"id": "T3", "type": "MINOR", "pos": (2, 2)},
            {"id": "T4", "type": "MINOR", "pos": (5, 5)}
        ]
        
    def random_pos(self):
        return (random.randint(0, self.grid_size-1), random.randint(0, self.grid_size-1))
        
    def randomize_obstacles(self, num_obstacles):
        self.obstacles.clear()
        for _ in range(num_obstacles):
             self.obstacles.add(self.random_pos())

    # ================================
    # COLLISION AVOIDANCE & MOVEMENT
    # ================================
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def step_simulation(self):
        self.metrics["steps_taken"] += 1
        intended_moves = {}
        
        # 1. Plan Moves toward respective tasks
        for bot_id, bot in self.bots.items():
            if bot["status"] != "ACTIVE" or not bot["tasks"]:
                intended_moves[bot_id] = bot["pos"] # Idle bots stay still
                continue
                
            current_task = bot["tasks"][0]
            target = current_task["pos"]
            
            # Check if task is reached
            if bot["pos"] == target:
                print(f"[!] Bot {bot_id} COMPLETED task {current_task['id']}!")
                self.metrics["tasks_completed"] += 1
                bot["tasks"].pop(0) # Remove completed task from workload
                intended_moves[bot_id] = bot["pos"]
                
                # Check for idle tasks to pick up
                self.assign_tasks()
                continue
            
            # Simple greedy pathing towards target (Manhattan step)
            x, y = bot["pos"]
            tx, ty = target
            next_x, next_y = x, y
            
            if x < tx: next_x += 1
            elif x > tx: next_x -= 1
            elif y < ty: next_y += 1
            elif y > ty: next_y -= 1
                
            if (next_x, next_y) not in self.obstacles:
                intended_moves[bot_id] = (next_x, next_y)
            else:
                intended_moves[bot_id] = bot["pos"] # Wait if blocked by a wall

        # 2. Collision Resolution Logic
        claimed = set()
        safe_moves = {}
        for bot_id, n_pos in intended_moves.items():
            # If two bots try to take the same tile, the second one aborts and waits
            if n_pos in claimed:
                safe_moves[bot_id] = self.bots[bot_id]["pos"]
                self.metrics["collisions_avoided"] += 1
            else:
                safe_moves[bot_id] = n_pos
                claimed.add(n_pos)
                
        # 3. Apply Safe Moves
        for bot_id, s_pos in safe_moves.items():
            self.bots[bot_id]["pos"] = s_pos


    # ================================
    # ADVANCED TASK REDISTRIBUTION
    # ================================
    def assign_tasks(self):
        """Initial baseline queue popping"""
        for bot_id, bot in self.bots.items():
            if bot["status"] == "ACTIVE" and len(bot["tasks"]) == 0 and self.pending_tasks:
                task = self.pending_tasks.pop(0)
                bot["tasks"].append(task)
                print(f"Bot {bot_id} initially assigned: {task['id']} ({task['type']})")

    def reassign_failed_task(self, failed_task):
        """
        Intelligent redistribution based on Distance and Workload rules.
        """
        self.metrics["reassignments_made"] += 1
        
        best_bot_id = None
        best_score = float('inf')  # Lower score wins for MAJOR tasks
        farthest_score = -1        # Higher score wins for MINOR tasks
        farthest_bot_id = None

        active_bots = {k: v for k, v in self.bots.items() if v["status"] == "ACTIVE"}
        if not active_bots:
            print("CRITICAL: No active bots alive to take the task!")
            self.pending_tasks.append(failed_task)
            return

        for bot_id, bot in active_bots.items():
            dist = self.heuristic(bot["pos"], failed_task["pos"])
            
            # Calculate Workload (Idle = 0 | Minor = 1 | Major = 2)
            workload = sum(2 if t["type"] == "MAJOR" else 1 for t in bot["tasks"])
            
            # RULE 1: If MAJOR task -> Target Nearest OR Idle bots
            if failed_task["type"] == "MAJOR":
                # Heavily penalizes high workloads. 
                # (e.g. Workload 0 + Dist 10 beats Workload 2 + Dist 2)
                score = dist + (workload * 30)
                if score < best_score:
                    best_score = score
                    best_bot_id = bot_id
                    
            # RULE 2: If MINOR task -> Give to ones FAR AWAY or with low workload
            else:
                # Maximizing this score picks far bots, but still slightly penalizes busy bots
                score = dist - (workload * 5) 
                if score > farthest_score:
                    farthest_score = score
                    farthest_bot_id = bot_id

        # Execute assignment based on the weight decision
        assigned_to = best_bot_id if failed_task["type"] == "MAJOR" else farthest_bot_id
        
        if assigned_to is not None:
            self.bots[assigned_to]["tasks"].append(failed_task)
            print(f"   -> SMART REASSIGN: Task {failed_task['id']} ({failed_task['type']}) given to Bot {assigned_to}!")

    def inject_failure(self, bot_id):
        print(f"\n[!!!] CRITICAL EVENT: Bot {bot_id} has crashed [!!!]")
        failed_bot = self.bots[bot_id]
        failed_bot["status"] = "OFFLINE"
        
        # Dump all of its current tasks and reassign them intelligently
        for stolen_task in failed_bot["tasks"]:
            print(f"Retrieving lost task: {stolen_task['id']}")
            self.reassign_failed_task(stolen_task)
            
        failed_bot["tasks"] = []

    def print_state(self):
        print("\n--- PERFORMANCE METRICS & STATE ---")
        for k, v in self.metrics.items():
            print(f" > {k}: {v}")
        for bot_id, bot in self.bots.items():
            task_list = [t["id"] for t in bot["tasks"]]
            print(f"Bot {bot_id} ({bot['status']}): Pos={bot['pos']} | Tasks {len(task_list)}: {task_list}")
            
# ==========================================
# TEST EXECUTION
# ==========================================
if __name__ == "__main__":
    system = MultiBotSystem()
    
    print("\n--- 1. INITIAL ASSIGNMENT ---")
    system.assign_tasks()
    
    print("\n--- 2. SIMULATING SEVERAL STEPS (Collision Checking) ---")
    for _ in range(5):
        system.step_simulation()
    
    system.print_state()
    
    print("\n--- 3. MID-SIMULATION FAILURE ---")
    # Identify a bot that actually has a task and crash it
    busy_bot = next((b for b in system.bots if system.bots[b]["tasks"] and system.bots[b]["status"] == "ACTIVE"), None)
    if busy_bot is not None:
        system.inject_failure(busy_bot)
    
    system.print_state()