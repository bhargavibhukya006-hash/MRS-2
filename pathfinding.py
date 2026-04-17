# pathfinding.py

import heapq

def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(pos, grid_size, obstacles):
    x, y = pos
    neighbors = []
    directions = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
    
    for nx, ny in directions:
        # Check bounds
        if 0 <= nx < grid_size and 0 <= ny < grid_size:
            # Check obstacles (Part 1)
            if (nx, ny) in obstacles:
                # Debug Logging (Part 6)
                # print(f"DEBUG [A*]: Skipping obstacle cell at ({nx}, {ny})")
                continue
            neighbors.append((nx, ny))
    return neighbors

def astar(start, goal, grid_size, obstacles):
    # STEP 1: Debug print — counts must match "[DEBUG] Obstacles in world: N" in main.py
    print(f"[DEBUG A*] Start: {start}, Goal: {goal}, Obstacles: {len(obstacles)}")

    # Hard guard: never search if start/goal are blocked — return no-path immediately
    if start in obstacles or goal in obstacles:
        print(f"[CRITICAL A*] start={start} or goal={goal} is inside obstacles — aborting search")
        return [start]

    if start == goal:
        return [start]
    
    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = {}
    g_score = {start: 0}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            # reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for neighbor in get_neighbors(current, grid_size, obstacles):
            tentative_g = g_score[current] + 1

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score, neighbor))

    return [start]  # no path found
