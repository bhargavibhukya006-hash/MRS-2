# visualization.py

import pygame
import sys
import math

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
DARK_GREEN = (0, 150, 0)
GRAY = (150, 150, 150)
LIGHT_GRAY = (200, 200, 200)
LIGHT_BLUE = (173, 216, 230)
GOLD = (255, 215, 0)

CELL_SIZE = 40

class Visualizer:
    def __init__(self, world):
        pygame.init()
        pygame.font.init()
        self.world = world  # Reference for shared config like obstacles
        self.grid_size = world.grid_size
        self.size = self.grid_size * CELL_SIZE
        self.width = self.size * 2
        self.screen = pygame.display.set_mode((self.width, self.size))
        pygame.display.set_caption("Multi-Agent Simulation: RULE vs PREDICT")
        
        self.font = pygame.font.SysFont("Arial", 14)
        self.title_font = pygame.font.SysFont("Arial", 24, bold=True)
        self.metric_font = pygame.font.SysFont("Arial", 18)
        self.clock = pygame.time.Clock()
        
        self.trails_rule = {}  # aid: [(r, c), ...]
        self.trails_pred = {}  # aid: [(r, c), ...]

    def draw_grid(self, offset_x):
        for x in range(0, self.size + 1, CELL_SIZE):
            pygame.draw.line(self.screen, LIGHT_GRAY, (x + offset_x, 0), (x + offset_x, self.size))
        for y in range(0, self.size + 1, CELL_SIZE):
            pygame.draw.line(self.screen, LIGHT_GRAY, (offset_x, y), (self.size + offset_x, y))

    def draw_obstacles(self, world, offset_x):
        for (x, y) in world.obstacles:
            pygame.draw.rect(
                self.screen,
                BLACK,
                (y * CELL_SIZE + offset_x, x * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            )

    def draw_target(self, world, offset_x):
        x, y = world.target_position
        pygame.draw.rect(
            self.screen,
            GREEN,
            (y * CELL_SIZE + offset_x, x * CELL_SIZE, CELL_SIZE, CELL_SIZE)
        )
        center = (y * CELL_SIZE + CELL_SIZE // 2 + offset_x, x * CELL_SIZE + CELL_SIZE // 2)
        pygame.draw.circle(self.screen, DARK_GREEN, center, CELL_SIZE // 3)

    def draw_paths(self, world, paths, offset_x):
        if not paths: return
        for aid, path in paths.items():
            if path and len(path) > 1 and world.agent_status.get(aid) != "BLOCKED":
                points = [(p[1] * CELL_SIZE + CELL_SIZE // 2 + offset_x, p[0] * CELL_SIZE + CELL_SIZE // 2) for p in path]
                pygame.draw.lines(self.screen, LIGHT_BLUE, False, points, 2)

    def draw_trails(self, trails, offset_x):
        for aid, trail in trails.items():
            if len(trail) > 1:
                for i in range(len(trail) - 1):
                    p1, p2 = trail[i], trail[i+1]
                    c1 = (int(p1[1] * CELL_SIZE + CELL_SIZE // 2 + offset_x), int(p1[0] * CELL_SIZE + CELL_SIZE // 2))
                    c2 = (int(p2[1] * CELL_SIZE + CELL_SIZE // 2 + offset_x), int(p2[0] * CELL_SIZE + CELL_SIZE // 2))
                    pygame.draw.line(self.screen, (200, 200, 200), c1, c2, 2)

    def _draw_agent(self, aid, center, status, role, n_pos, o_pos):
        colors = [(255, 0, 0), (0, 0, 255), (255, 165, 0), (128, 0, 128)]
        is_failed = (status == "BLOCKED")
        color = GRAY if is_failed else colors[aid % len(colors)]

        pygame.draw.circle(self.screen, color, center, CELL_SIZE // 3)

        if is_failed:
            pygame.draw.line(self.screen, RED, (center[0]-8, center[1]-8), (center[0]+8, center[1]+8), 3)
            pygame.draw.line(self.screen, RED, (center[0]+8, center[1]-8), (center[0]-8, center[1]+8), 3)
        else:
            if n_pos != o_pos:
                dx, dy = n_pos[1] - o_pos[1], n_pos[0] - o_pos[0]
                angle = math.atan2(dy, dx)
                end_x = center[0] + int(math.cos(angle) * (CELL_SIZE // 3))
                end_y = center[1] + int(math.sin(angle) * (CELL_SIZE // 3))
                pygame.draw.line(self.screen, BLACK, center, (end_x, end_y), 2)

        role_surf = self.font.render(role, True, BLACK)
        self.screen.blit(role_surf, (center[0] - role_surf.get_width()//2, center[1] - CELL_SIZE//2))

    def draw_metrics(self, offset_x, metrics, mode_name, improvement=None):
        title = self.title_font.render(mode_name, True, BLACK)
        self.screen.blit(title, (offset_x + 10, 10))
        stats_text = [
            f"Steps: {metrics['steps']}",
            f"Collisions Avoided: {metrics['collisions']}",
            f"Wait Actions: {metrics['waits']}"
        ]
        for i, text in enumerate(stats_text):
            surf = self.metric_font.render(text, True, BLACK)
            self.screen.blit(surf, (offset_x + 10, 40 + i * 20))
        if improvement is not None:
            imp_text = f"Efficiency Gain: {improvement:.1f}%"
            imp_surf = self.metric_font.render(imp_text, True, DARK_GREEN)
            self.screen.blit(imp_surf, (offset_x + self.size - imp_surf.get_width() - 10, 10))

    def animate_two_worlds(self, w_rule, old_rule, new_rule, paths_rule, metrics_rule,
                           w_pred, old_pred, new_pred, paths_pred, metrics_pred,
                           improvement, failed_aid=None):
        frames = 15
        for world_type in ['rule', 'pred']:
            trails = self.trails_rule if world_type == 'rule' else self.trails_pred
            positions = new_rule if world_type == 'rule' else new_pred
            for aid in positions:
                if aid not in trails: trails[aid] = []

        for frame in range(frames + 1):
            alpha = frame / frames
            for event in pygame.event.get():
                if event.type == pygame.QUIT: pygame.quit(); sys.exit()

            self.screen.fill(WHITE)
            
            # RULE side
            self.draw_grid(0)
            self.draw_obstacles(w_rule, 0)
            self.draw_trails(self.trails_rule, 0)
            self.draw_paths(w_rule, paths_rule, 0)
            self.draw_target(w_rule, 0)
            for aid, n_pos in new_rule.items():
                o_pos = old_rule.get(aid, n_pos)
                interp = (o_pos[0] + alpha*(n_pos[0]-o_pos[0]), o_pos[1] + alpha*(n_pos[1]-o_pos[1]))
                center = (int(interp[1]*CELL_SIZE + CELL_SIZE//2), int(interp[0]*CELL_SIZE + CELL_SIZE//2))
                self._draw_agent(aid, center, w_rule.agent_status[aid], w_rule.agent_roles[aid], n_pos, o_pos)
            self.draw_metrics(0, metrics_rule, "RULE MODE")

            # PREDICT side
            offset = self.size
            self.draw_grid(offset)
            self.draw_obstacles(w_pred, offset)
            self.draw_trails(self.trails_pred, offset)
            self.draw_paths(w_pred, paths_pred, offset)
            self.draw_target(w_pred, offset)
            for aid, n_pos in new_pred.items():
                o_pos = old_pred.get(aid, n_pos)
                interp = (o_pos[0] + alpha*(n_pos[0]-o_pos[0]), o_pos[1] + alpha*(n_pos[1]-o_pos[1]))
                center = (int(interp[1]*CELL_SIZE + CELL_SIZE//2 + offset), int(interp[0]*CELL_SIZE + CELL_SIZE//2))
                self._draw_agent(aid, center, w_pred.agent_status[aid], w_pred.agent_roles[aid], n_pos, o_pos)
            self.draw_metrics(offset, metrics_pred, "PREDICT MODE", improvement)

            pygame.draw.line(self.screen, BLACK, (self.size, 0), (self.size, self.size), 2)
            if failed_aid is not None:
                alert = self.title_font.render("FAILURE DETECTED", True, RED)
                self.screen.blit(alert, (self.width//2 - alert.get_width()//2, self.size - 40))
            pygame.display.flip()
            self.clock.tick(60)

        for (trails, pos_map) in [(self.trails_rule, new_rule), (self.trails_pred, new_pred)]:
            for aid, pos in pos_map.items():
                if not trails[aid] or trails[aid][-1] != pos: trails[aid].append(pos)
                if len(trails[aid]) > 10: trails[aid].pop(0)

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: pygame.quit(); sys.exit()
        pygame.display.flip()


