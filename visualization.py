# visualization.py

import pygame
import sys
import math
import random
import config

class Visualizer:
    def __init__(self, world):
        pygame.init()
        pygame.font.init()
        self.world = world
        self.grid_size = world.grid_size
        self.arena_size = self.grid_size * config.CELL_SIZE
        self.width = self.arena_size + config.SIDE_PANEL_WIDTH
        self.screen = pygame.display.set_mode((self.width, self.arena_size))
        pygame.display.set_caption("High-Fidelity Robotics Simulation")
        
        self.font = pygame.font.SysFont("Verdana", 14)
        self.title_font = pygame.font.SysFont("Verdana", 20, bold=True)
        self.hud_font = pygame.font.SysFont("Consolas", 16)
        
        self.clock = pygame.time.Clock()
        self.trails = {} # aid: [pos, ...]

    def draw_grid(self):
        for x in range(0, self.arena_size + 1, config.CELL_SIZE):
            pygame.draw.line(self.screen, config.COLOR_GRID, (x, 0), (x, self.arena_size), 1)
        for y in range(0, self.arena_size + 1, config.CELL_SIZE):
            pygame.draw.line(self.screen, config.COLOR_GRID, (0, y), (self.arena_size, y), 1)

    def draw_obstacles(self):
        for (r, c) in self.world.obstacles:
            center = (c * config.CELL_SIZE + config.CELL_SIZE // 2, r * config.CELL_SIZE + config.CELL_SIZE // 2)
            # Draw obstacles as soft cylinders/circles
            pygame.draw.circle(self.screen, config.COLOR_OBSTACLE, center, config.CELL_SIZE // 2 - 2)
            pygame.draw.circle(self.screen, (80, 90, 110), center, config.CELL_SIZE // 2 - 2, 2)

    def draw_target(self):
        r, c = self.world.target_position
        rect = (c * config.CELL_SIZE + 2, r * config.CELL_SIZE + 2, config.CELL_SIZE - 4, config.CELL_SIZE - 4)
        pygame.draw.rect(self.screen, (0, 100, 0), rect, 0, 4) # Rounded target base
        center = (c * config.CELL_SIZE + config.CELL_SIZE // 2, r * config.CELL_SIZE + config.CELL_SIZE // 2)
        pygame.draw.circle(self.screen, config.COLOR_GREEN, center, config.CELL_SIZE // 4)

    def draw_connection_beam(self):
        # Draw beam between PRIMARY and SECONDARY carriers when near target
        primary_pos = None
        secondary_pos = None
        for aid, role in self.world.agent_roles.items():
            if role == "PRIMARY_CARRIER": primary_pos = self.world.agent_positions[aid]
            if role == "SECONDARY_CARRIER": secondary_pos = self.world.agent_positions[aid]
            
        if primary_pos and secondary_pos:
            target = self.world.target_position
            dist_p = abs(primary_pos[0] - target[0]) + abs(primary_pos[1] - target[1])
            dist_s = abs(secondary_pos[0] - target[0]) + abs(secondary_pos[1] - target[1])
            
            if dist_p < 5 and dist_s < 5:
                p_center = (primary_pos[1] * config.CELL_SIZE + config.CELL_SIZE // 2, primary_pos[0] * config.CELL_SIZE + config.CELL_SIZE // 2)
                s_center = (secondary_pos[1] * config.CELL_SIZE + config.CELL_SIZE // 2, secondary_pos[0] * config.CELL_SIZE + config.CELL_SIZE // 2)
                pygame.draw.line(self.screen, (0, 255, 255, 128), p_center, s_center, 3)

    def _draw_agent(self, aid, center, role, is_waiting, status):
        is_blocked = (status == config.STATUS_BLOCKED)
        colors = [config.COLOR_ACCENT, (100, 200, 255), (200, 100, 255)]
        color = config.GRAY if is_blocked else colors[aid % len(colors)]
        
        # Jitter effect for waiting/micro-adjustments
        render_center = center
        if is_waiting and not is_blocked:
            render_center = (center[0] + random.randint(-1, 1), center[1] + random.randint(-1, 1))

        # Agent Body
        pygame.draw.circle(self.screen, color, render_center, config.CELL_SIZE // 3)
        if role in ["PRIMARY_CARRIER", "SECONDARY_CARRIER"]:
            pygame.draw.circle(self.screen, config.COLOR_GOLD, render_center, config.CELL_SIZE // 3 + 2, 2)

        # Heading Line
        if not is_blocked:
            target = self.world.target_position
            curr_pos = self.world.agent_positions[aid]
            angle = math.atan2(target[0] - curr_pos[0], target[1] - curr_pos[1])
            end_x = render_center[0] + math.cos(angle) * (config.CELL_SIZE // 3)
            end_y = render_center[1] + math.sin(angle) * (config.CELL_SIZE // 3)
            pygame.draw.line(self.screen, config.COLOR_WHITE, render_center, (end_x, end_y), 2)

    def draw_side_panel(self, metrics, comp_pct):
        panel_rect = (self.arena_size, 0, config.SIDE_PANEL_WIDTH, self.arena_size)
        pygame.draw.rect(self.screen, config.COLOR_PANEL, panel_rect)
        pygame.draw.line(self.screen, config.COLOR_GRID, (self.arena_size, 0), (self.arena_size, self.arena_size), 2)
        
        # Header
        title = self.title_font.render("SYSTEM TELEMETRY", True, config.COLOR_ACCENT)
        self.screen.blit(title, (self.arena_size + 20, 20))
        
        # Completion Bar
        comp_title = self.font.render(f"MISSION COMPLETION: {comp_pct}%", True, config.COLOR_WHITE)
        self.screen.blit(comp_title, (self.arena_size + 20, 70))
        pygame.draw.rect(self.screen, config.COLOR_GRID, (self.arena_size + 20, 95, 210, 15))
        pygame.draw.rect(self.screen, config.COLOR_GREEN, (self.arena_size + 20, 95, 2.1 * comp_pct, 15))
        
        # Agent Roles
        y_offset = 150
        role_header = self.title_font.render("AGENT ROLES", True, config.COLOR_WHITE)
        self.screen.blit(role_header, (self.arena_size + 20, y_offset))
        y_offset += 40
        
        for aid in range(self.world.num_agents):
            role = self.world.agent_roles.get(aid, "UNASSIGNED")
            status = self.world.agent_status.get(aid, "ACTIVE")
            color = config.COLOR_GREEN if status == "ACTIVE" else config.COLOR_RED
            
            role_text = self.hud_font.render(f"AGENT {aid}: {role}", True, config.COLOR_WHITE)
            self.screen.blit(role_text, (self.arena_size + 20, y_offset))
            status_dot = pygame.draw.circle(self.screen, color, (self.arena_size + 220, y_offset + 10), 6)
            y_offset += 30

        # Statistics
        stats_header = self.title_font.render("STATISTICS", True, config.COLOR_WHITE)
        self.screen.blit(stats_header, (self.arena_size + 20, y_offset + 50))
        
        stats = [
            f"Steps Taken: {metrics['steps']}",
            f"Wait Actions: {metrics['waits']}",
            f"Collisions Avoided: {metrics['collisions']}"
        ]
        for i, text in enumerate(stats):
            surf = self.hud_font.render(text, True, config.COLOR_GOLD)
            self.screen.blit(surf, (self.arena_size + 20, y_offset + 90 + i * 25))

    def animate_high_fidelity(self, world, old_pos, new_pos, paths, metrics, comp_pct):
        frames = 20
        # Check for waiting status
        waits = {aid: (old_pos[aid] == new_pos[aid] and world.agent_status[aid] != config.STATUS_BLOCKED) for aid in old_pos}
        
        for frame in range(frames + 1):
            alpha = frame / frames
            for event in pygame.event.get():
                if event.type == pygame.QUIT: pygame.quit(); sys.exit()

            self.screen.fill(config.COLOR_BG)
            self.draw_grid()
            self.draw_obstacles()
            self.draw_target()
            self.draw_connection_beam()
            
            # Draw Path markers (faded)
            if paths:
                for aid, path in paths.items():
                    if path and len(path) > 1:
                        points = [(p[1] * config.CELL_SIZE + config.CELL_SIZE//2, p[0] * config.CELL_SIZE + config.CELL_SIZE//2) for p in path]
                        if len(points) > 1:
                            pygame.draw.lines(self.screen, (40, 60, 80), False, points, 2)

            # Draw Interp Agents
            for aid, n_pos in new_pos.items():
                o_pos = old_pos.get(aid, n_pos)
                interp = (o_pos[0] + alpha*(n_pos[0]-o_pos[0]), o_pos[1] + alpha*(n_pos[1]-o_pos[1]))
                center = (int(interp[1] * config.CELL_SIZE + config.CELL_SIZE // 2), 
                          int(interp[0] * config.CELL_SIZE + config.CELL_SIZE // 2))
                self._draw_agent(aid, center, world.agent_roles[aid], waits[aid], world.agent_status[aid])
                
            self.draw_side_panel(metrics, comp_pct)
            
            if metrics["task_completed"]:
                overlay = pygame.Surface((self.width, self.arena_size), pygame.SRCALPHA)
                overlay.fill((0, 0, 0, 150))
                self.screen.blit(overlay, (0, 0))
                msg = self.title_font.render("MISSION ACCOMPLISHED", True, config.COLOR_GREEN)
                self.screen.blit(msg, (self.width // 2 - msg.get_width() // 2, self.arena_size // 2))

            pygame.display.flip()
            self.clock.tick(config.FPS)

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); sys.exit()
        pygame.display.flip()