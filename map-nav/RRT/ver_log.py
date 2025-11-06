import json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os
import numpy as np
from typing import List, Dict, Any, Optional

MAP_FILENAME: str = "simulation_map.pgm"
LOG_FILENAME: str = "simulation_log.json"
METADATA_FILENAME: str = "map_metadata.json"

def load_json_file(filename: str) -> Optional[Dict]:
    """Carrega um arquivo JSON genérico."""
    if not os.path.exists(filename):
        print(f"Erro: Arquivo não encontrado em '{filename}'")
        return None
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
        print(f"Dados carregados com sucesso de '{filename}'.")
        return data
    except Exception as e:
        print(f"Erro ao carregar ou decodificar JSON de '{filename}': {e}")
        return None

def load_map_image(filename: str) -> Optional[np.ndarray]:
    """Carrega a imagem do mapa a partir de um arquivo PGM."""
    if not os.path.exists(filename):
        print(f"Erro: Arquivo do mapa não encontrado em '{filename}'")
        return None
    try:
        map_img = mpimg.imread(filename)
        print(f"Imagem do mapa carregada de '{filename}'. Dimensões: {map_img.shape[1]}x{map_img.shape[0]} pixels.")
        return map_img
    except Exception as e:
        print(f"Erro ao ler o arquivo do mapa '{filename}': {e}")
        return None

def visualize_simulation(log_data: List[Dict[str, Any]], map_img: Optional[np.ndarray], map_metadata: Dict) -> None:
    """Cria um gráfico para visualizar a trajetória do robô no mapa."""
    if not log_data:
        print("Nenhum dado de log para visualizar.")
        return

    poses = [entry.get('pose', {}) for entry in log_data]
    path_x = [p.get('x') for p in poses if p.get('x') is not None]
    path_y = [p.get('y') for p in poses if p.get('y') is not None]

    if not path_x or not path_y:
        print("Erro: Nenhum dado de pose (coordenadas x, y) válido encontrado no log.")
        return

    plt.figure(figsize=(12, 10))

    if map_img is not None and map_metadata:
        try:
            mx_min = map_metadata["world_x_min"]
            my_min = map_metadata["world_y_min"]
            res = map_metadata["resolution"]
            border = map_metadata["border_pixels"]
            map_w = map_metadata["map_width_pixels"]
            map_h = map_metadata["map_height_pixels"]
            
            center_x0 = mx_min - border * res
            center_y0 = my_min - border * res
            
            ext_left = center_x0 - (res / 2.0)
            ext_bottom = center_y0 - (res / 2.0)
            
            center_xN = mx_min + (map_w - 1 - border) * res
            center_yN = my_min + (map_h - 1 - border) * res
            
            ext_right = center_xN + (res / 2.0)
            ext_top = center_yN + (res / 2.0)

            extent = [ext_left, ext_right, ext_bottom, ext_top]
            
            plt.imshow(map_img, cmap='gray', extent=extent, origin='upper')
            
            print(f"Mapa plotado com extent (coordenadas do mundo): {extent}")
            
        except KeyError as e:
            print(f"Erro: Metadados incompletos (execute rrt_planner.py novamente). Faltando: {e}")
            plt.imshow(map_img, cmap='gray', origin='upper') # Fallback
        except Exception as e:
            print(f"Erro ao processar metadados do mapa: {e}")
            plt.imshow(map_img, cmap='gray', origin='upper') # Fallback
    else:
        print("Mapa ou metadados não carregados. Plotando apenas as trajetórias.")

    plt.plot(path_x, path_y, marker='.', markersize=2, linestyle='-', color='red', label='Trajetória do Robô', zorder=5)

    replanning_points = [(e['pose']['x'], e['pose']['y']) for e in log_data if 'REPLANNING' in e.get('state', '') and e.get('pose')]
    if replanning_points:
        repl_x, repl_y = zip(*replanning_points)
        plt.scatter(repl_x, repl_y, c='orange', s=80, label='Replanejamento', zorder=7, edgecolors='black', marker='v')
        print(f"Destacados {len(replanning_points)} eventos de replanejamento.")

    if log_data and 'dynamic_obstacles' in log_data[0] and log_data[0]['dynamic_obstacles']:
        num_dyn_obs = len(log_data[0]['dynamic_obstacles'])
        obs_trails = {i: ([], []) for i in range(num_dyn_obs)}
        for entry in log_data:
            for i, obs in enumerate(entry['dynamic_obstacles']):
                obs_trails[i][0].append(obs['x'])
                obs_trails[i][1].append(obs['y'])
        
        colors = plt.cm.viridis(np.linspace(0, 1, num_dyn_obs))
        for i, (trail_x, trail_y) in obs_trails.items():
            plt.plot(trail_x, trail_y, linestyle='--', color=colors[i], alpha=0.7, label=f'Trilha Obst. Din. {i+1}', zorder=3)
            plt.gca().add_patch(plt.Circle((trail_x[-1], trail_y[-1]), log_data[-1]['dynamic_obstacles'][i]['size'], color=colors[i], alpha=0.5, zorder=4))
        print(f"Plotadas as trilhas de {num_dyn_obs} obstáculos dinâmicos.")


    plt.scatter(path_x[0], path_y[0], c='lime', s=150, marker='o', label='Início', zorder=6, edgecolors='black')
    plt.scatter(path_x[-1], path_y[-1], c='cyan', s=150, marker='X', label='Fim', zorder=6, edgecolors='black')
    plt.xlabel('Coordenada X (Mundo)')
    plt.ylabel('Coordenada Y (Mundo)')
    plt.title('Análise da Simulação RRT Dinâmico')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0))
    plt.tight_layout(rect=[0, 0, 0.85, 1])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

if __name__ == "__main__":
    print("--- Analisador de Log da Simulação ---")
    log_data = load_json_file(LOG_FILENAME)
    map_image = load_map_image(MAP_FILENAME)
    map_meta = load_json_file(METADATA_FILENAME)

    if log_data and map_meta:
        visualize_simulation(log_data, map_image, map_meta)
    else:
        print("Saindo: Dados de log ou metadados do mapa não puderam ser carregados.")

    print("--- Análise Concluída ---")