# ECOM140 - Navegação de Robôs

Projeto desenvolvido para a disciplina **ECOM140 - Navegação de Robôs**, com foco em algoritmos de navegação autônoma e planejamento de trajetórias em ambientes simulados.

## Descrição

O repositório está dividido em duas abordagens principais de navegação e planejamento:

- **map-nav/** — Contém as implementações dos algoritmos **D\* Lite** e **RRT**, desenvolvidos em **Python** utilizando as bibliotecas **matplotlib** e **numpy** para simulação e visualização dos resultados.  
- **react-nav/** — Implementa uma solução de **navegação reativa** baseada em **ROS2**, utilizando o modelo 3D do **TurtleBot Waffle** e arquivos **rosbag** para gravação e visualização de logs de navegação.  

Cada uma dessas pastas possui seu próprio **README.md**, com detalhes específicos sobre execução e funcionamento.

## Estrutura do Projeto

```
ECOM140-Navegacao-de-Robos/
├── map-nav/
│ ├── DStarLite/ # Algoritmo D* Lite para planejamento dinâmico
│ ├── RRT/ # Algoritmo RRT (Rapidly-Exploring Random Tree)
│ └── README.md
│
├── react-nav/ # Navegação reativa com ROS2 e TurtleBot Waffle
│ ├── images/
│ ├── models/
│ ├── scenes/
│ ├── src/reactive_nav/
│ └── README.md
│
└── README.md
```

## Obs

Para executar os testes dos algoritmos de map-nav, é importante usar um ambiente virtual python para evitar conflitos das bilbiotecas utilizadas.

```
cd map-nav
python3 -m venv venv
source venv/bin/activate  # No Windows: venv\Scripts\activate
pip install numpy matplotlib
```

### RRT

```
cd 
python3 rrt_planner.py
```


```
python3 ver_log.py      # Exibe informações de log da simulação
python3 ver_mapa.py     # Mostra o mapa com o caminho encontrado
```

### D*Lite

```
cd ../DStarLite
python3 d_star_lite.py
```

```
python3 ver_log.py
python3 ver_mapa.py
```



## Autor

- Rogério Filho
- Renalvo Alves