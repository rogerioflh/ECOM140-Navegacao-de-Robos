# rrt_planner.py
# -*- coding: utf-8 -*-

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches  # Importado para desenhar retângulos
import random
import math
import numpy as np
import json
import time

# --- Constantes para Geração de Mapa (IMPORTANTE PARA ALINHAMENTO) ---
MAP_RESOLUTION = 0.25
MAP_BORDER = 2

class RRT:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start, goal, obstacle_list, rand_area,
                 expand_dis=3.0, path_resolution=0.5, goal_sample_rate=5,
                 max_iter=500):
        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        # A lista de obstáculos agora pode conter diferentes tipos, mas vamos focar nos retangulares
        self.obstacle_list = obstacle_list
        self.node_list = []

    def planning(self, dynamic_obstacle_list=None):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
            # A checagem de colisão foi atualizada para retângulos
            if self.check_collision(new_node, dynamic_obstacle_list):
                self.node_list.append(new_node)
            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.goal, self.expand_dis)
                if self.check_collision(final_node, dynamic_obstacle_list):
                    self.node_list.append(final_node)
                    return self.generate_final_course(len(self.node_list) - 1)
        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        if extend_length > d:
            extend_length = d
        n_expand = math.floor(extend_length / self.path_resolution)
        for _ in range(int(n_expand)):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
        new_node.parent = from_node
        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.goal.x
        dy = y - self.goal.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:
            rnd = self.Node(self.goal.x, self.goal.y)
        return rnd
    
    # ### FUNÇÃO DE COLISÃO MODIFICADA ###
    def check_collision(self, node, dynamic_obstacle_list):
        if node is None:
            return False
            
        # Checa colisão com obstáculos retangulares estáticos
        for (ox, oy, width, height) in self.obstacle_list:
            for (ix, iy) in zip(node.path_x, node.path_y):
                if ox <= ix <= ox + width and oy <= iy <= oy + height:
                    return False  # Colisão
                    
        # Checa colisão com obstáculos dinâmicos (ainda circulares, pode ser adaptado)
        if dynamic_obstacle_list:
            for obs in dynamic_obstacle_list:
                if obs.check_node_path_collision(node.path_x, node.path_y):
                    return False
        return True # Sem colisão

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2
                 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

class DynamicObstacle: # Mantido como círculo por enquanto
    def __init__(self, x, y, size, speed_x=0, speed_y=0):
        self.x = x
        self.y = y
        self.size = size
        self.speed_x = speed_x
        self.speed_y = speed_y

    def update_position(self, dt):
        self.x += self.speed_x * dt
        self.y += self.speed_y * dt

    def check_node_path_collision(self, path_x, path_y):
        for ix, iy in zip(path_x, path_y):
            if math.hypot(self.x - ix, self.y - iy) <= self.size:
                return True
        return False

    def check_segment_collision(self, p1, p2):
        num_checks = 10
        for i in range(num_checks + 1):
            t = i / num_checks
            px = p1[0] + t * (p2[0] - p1[0])
            py = p1[1] + t * (p2[1] - p1[1])
            if math.hypot(self.x - px, self.y - py) <= self.size:
                return True
        return False
        
    def to_dict(self):
        return {"x": self.x, "y": self.y, "size": self.size}

# ### FUNÇÃO DE DESENHO MODIFICADA ###
def draw_simulation_state(current_x, current_y, current_path, static_obstacles, dynamic_obstacles, rand_area):
    plt.clf()
    ax = plt.gca() # Pega o eixo atual para adicionar os retângulos
    
    # Desenha obstáculos estáticos (retângulos)
    for (ox, oy, width, height) in static_obstacles:
        rect = patches.Rectangle((ox, oy), width, height, linewidth=1, edgecolor='black', facecolor='black')
        ax.add_patch(rect)

    # Desenha obstáculos dinâmicos (círculos)
    for obs in dynamic_obstacles:
        ax.add_patch(plt.Circle((obs.x, obs.y), obs.size, color='orange', alpha=0.7))
    
    # Desenha o robô e a rota planejada
    plt.plot(current_x, current_y, "Dg", markersize=10, label="Robô")
    if current_path:
        path_x, path_y = zip(*current_path)
        plt.plot(path_x, path_y, '-r', label="Rota Planejada")
    
    plt.plot(x_start[0], x_start[1], "go", markersize=12, label="Início")
    plt.plot(x_goal[0], x_goal[1], "rx", markersize=12, label="Objetivo")

    plt.axis("equal")
    plt.grid(True)
    plt.xlim(rand_area[0], rand_area[1])
    plt.ylim(rand_area[0], rand_area[1])
    plt.legend()
    plt.title(f"Simulação RRT Dinâmico")
    plt.pause(0.01)

# ### FUNÇÃO DE GERAR MAPA MODIFICADA ###
def create_static_map_pgm(filename, rand_area, static_obs_rect, dynamic_obs_initial_circ):
    """Cria e salva um mapa PGM com o estado inicial dos obstáculos."""
    world_x_min, world_y_min = rand_area[0], rand_area[0]
    world_x_max, world_y_max = rand_area[1], rand_area[1]
    
    map_width = int((world_x_max - world_x_min) / MAP_RESOLUTION) + 1 + 2 * MAP_BORDER
    map_height = int((world_y_max - world_y_min) / MAP_RESOLUTION) + 1 + 2 * MAP_BORDER
    grid_map = np.full((map_height, map_width), 254, dtype=np.uint8)

    for y_pix in range(map_height):
        for x_pix in range(map_width):
            world_x = world_x_min + (x_pix - MAP_BORDER) * MAP_RESOLUTION
            world_y = world_y_min + (y_pix - MAP_BORDER) * MAP_RESOLUTION
            
            # Checa obstáculos retangulares
            for (ox, oy, w, h) in static_obs_rect:
                if (ox <= world_x <= ox + w) and (oy <= world_y <= oy + h):
                    grid_map[y_pix, x_pix] = 0 # Ocupado
                    break
            if grid_map[y_pix, x_pix] == 0: continue

            # Checa obstáculos circulares dinâmicos (na posição inicial)
            for obs in dynamic_obs_initial_circ:
                if math.hypot(world_x - obs.x, world_y - obs.y) <= obs.size:
                    grid_map[y_pix, x_pix] = 0 # Ocupado
                    break

    with open(filename, 'wb') as f:
        header = f'P5\n{map_width} {map_height}\n255\n'
        f.write(header.encode('ascii'))
        f.write(np.flipud(grid_map).tobytes())
    
    print(f"Mapa estático salvo em '{filename}' ({map_width}x{map_height} pixels)")
    return {
        "world_x_min": world_x_min, "world_y_min": world_y_min,
        "map_width_pixels": map_width, "map_height_pixels": map_height
    }

# ### CONFIGURAÇÃO DO MAPA ATUALIZADA ###
# Área de simulação e pontos de início/fim
rand_area = [-2, 18]
x_start = [1.0, 1.0]
x_goal = [1.0, 13.0]

# Lista de obstáculos estáticos retangulares (x, y, largura, altura)
# Coordenadas aproximadas baseadas na sua imagem
rectangular_obstacles = [
    # Bordas do mapa
    (rand_area[0], rand_area[0], rand_area[1] - rand_area[0], 0.5),  # Borda inferior
    (rand_area[0], rand_area[1] - 0.5, rand_area[1] - rand_area[0], 0.5), # Borda superior
    (rand_area[0], rand_area[0], 0.5, rand_area[1] - rand_area[0]),  # Borda esquerda
    (rand_area[1] - 0.5, rand_area[0], 0.5, rand_area[1] - rand_area[0]), # Borda direita
    # Obstáculos internos
    (2, 9, 4, 4),
    (8, 8, 2, 7),
    (8, 2, 5, 4),
    (3, 2, 2, 4),
    (2, 6, 4, 0.5), # Obstáculo fino horizontal
    (1, 0, 15, 0.5) # Plataforma inferior
]

def main():
    print("Iniciando simulação RRT Dinâmico com mapa retangular...")
    simulation_log = []
    
    dynamic_obstacles = [
        DynamicObstacle(13.0, 13.0, 1.0, speed_x=-0.2, speed_y=-0.1),
        DynamicObstacle(7.0, 6.0, 0.8, speed_x=0.1, speed_y=0.15)
    ]

    map_metadata = create_static_map_pgm("simulation_map.pgm", rand_area, rectangular_obstacles, dynamic_obstacles)

    current_x, current_y = x_start
    dt = 0.1
    current_path = []
    
    plt.ion()
    try:
        try:
            simulation_time = 0.0
            while math.hypot(current_x - x_goal[0], current_y - x_goal[1]) > 0.5:
                for obs in dynamic_obstacles:
                    obs.update_position(dt)
                    if not (rand_area[0] < obs.x < rand_area[1]): obs.speed_x *= -1
                    if not (rand_area[0] < obs.y < rand_area[1]): obs.speed_y *= -1

                replan = False
                if not current_path:
                    replan = True
                    state = "REPLANNING_NO_PATH"
                else:
                    state = "MOVING"
                    next_point = current_path[0] if len(current_path) == 1 else current_path[1]
                    for obs in dynamic_obstacles:
                        if obs.check_segment_collision((current_x, current_y), next_point):
                            replan = True
                            state = "REPLANNING_DETECTED"
                            print("Colisão detectada! Replanejando...")
                            break
                
                if replan:
                    rrt = RRT(start=[current_x, current_y], goal=x_goal, rand_area=rand_area,
                              obstacle_list=rectangular_obstacles, max_iter=1000)
                    path_nodes = rrt.planning(dynamic_obstacle_list=dynamic_obstacles)
                    if path_nodes:
                        current_path = path_nodes
                        current_path.reverse()
                    else:
                        current_path = []
                        print("Não foi possível encontrar uma rota. Tentando novamente...")
                
                log_entry = {
                    "timestamp": round(simulation_time, 2),
                    "pose": {"x": current_x, "y": current_y},
                    "state": state,
                    "dynamic_obstacles": [obs.to_dict() for obs in dynamic_obstacles]
                }
                simulation_log.append(log_entry)

                if current_path:
                    target_x, target_y = current_path[0]
                    angle = math.atan2(target_y - current_y, target_x - current_x)
                    robot_speed = 3.0
                    current_x += robot_speed * dt * math.cos(angle)
                    current_y += robot_speed * dt * math.sin(angle)
                    if math.hypot(current_x - target_x, current_y - target_y) < 0.3:
                        current_path.pop(0)

                draw_simulation_state(current_x, current_y, current_path, rectangular_obstacles, dynamic_obstacles, rand_area)
                simulation_time += dt

                if simulation_time > 150:
                     print("Limite de tempo da simulação atingido.")
                     break
            print("Objetivo alcançado!")

        except KeyboardInterrupt:
            print("\nSimulação interrompida pelo usuário.")

    finally:
        plt.ioff()
        if not simulation_log:
            print("Nenhum dado de log foi gerado para salvar.")
        else:
            print("Salvando log da simulação...")
            with open("simulation_log.json", "w") as f:
                json.dump(simulation_log, f, indent=4)
            print(f"Log salvo em 'simulation_log.json'. {len(simulation_log)} entradas.")
            
            with open("map_metadata.json", "w") as f:
                json.dump(map_metadata, f, indent=4)
        
        print("Simulação finalizada. Feche a janela do gráfico para encerrar.")
        plt.show()

if __name__ == '__main__':
    main()