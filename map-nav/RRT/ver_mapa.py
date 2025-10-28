# ver_mapa.py
# -*- coding: utf-8 -*-
"""
Visualiza o arquivo de mapa estático (simulation_map.pgm).
"""
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

MAP_FILENAME = "simulation_map.pgm"

def visualizar_mapa():
    """Carrega e exibe a imagem do mapa PGM."""
    try:
        print(f"Tentando carregar o mapa de '{MAP_FILENAME}'...")
        img = mpimg.imread(MAP_FILENAME)
        
        plt.figure(figsize=(8, 8))
        plt.imshow(img, cmap='gray', origin='lower') # 'origin' para corresponder ao sistema de coordenadas
        plt.title(f'Mapa Estático Inicial ({MAP_FILENAME})')
        plt.xlabel('Pixels (X)')
        plt.ylabel('Pixels (Y)')
        print("Mapa carregado com sucesso. Exibindo...")
        plt.show()
        
    except FileNotFoundError:
        print(f"Erro: Arquivo do mapa '{MAP_FILENAME}' não encontrado.")
        print("Por favor, execute primeiro o script 'rrt_planner.py' para gerar o mapa.")
    except Exception as e:
        print(f"Ocorreu um erro ao carregar ou exibir o mapa PGM: {e}")

if __name__ == "__main__":
    visualizar_mapa()