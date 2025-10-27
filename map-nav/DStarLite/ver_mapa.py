import matplotlib.pyplot as plt
import matplotlib.image as mpimg

try:
    img = mpimg.imread('simulation_map.pgm')
    plt.imshow(img, cmap='gray')
    plt.title('Mapa Estático (simulation_map.pgm)')
    plt.show()
except FileNotFoundError:
    print("Erro: Arquivo simulation_map.pgm não encontrado.")
except Exception as e:
    print(f"Erro ao carregar ou exibir o mapa PGM: {e}")