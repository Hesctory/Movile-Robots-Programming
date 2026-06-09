# PRM 2026 — Robótica Móvel

Workspace ROS 2 para a disciplina de Programação de Robôs Móveis 2026.

## Estrutura do projeto

```
src/
├── prm_2026/            # Pacote base da disciplina (submódulo externo)
│   └── launch/
│       ├── inicia_simulacao.launch.py   # Inicia o Gazebo + mundo
│       └── carrega_robo.launch.py       # Carrega o robô na simulação
│
├── robot_control/       # Nó de controle e navegação autônoma
│   └── launch/
│       └── robot_controller.launch.py  # Inicia os nós da equipe
│
└── robot_mapper/        # Nó de mapeamento por grade de ocupação
```

## Pacotes da equipe

### `robot_control`
Controle autônomo do robô com máquina de estados:
- **EXPLORING** — spin inicial, avanço com desvio de obstáculos (histerese) e perseguição do último rumo conhecido da bandeira
- **FLAG_DETECTED** — confirmação de detecção da bandeira pela câmera
- **NAVIGATING_TO_FLAG** — planejamento de caminho A* até a bandeira
- **POSITION_TO_COLLECT** — aproximação fina para coleta

Tópicos consumidos: `/scan`, `/robot_cam/labels_map`, `/model/prm_robot/pose`, `/grid_map`, `/imu`  
Tópicos publicados: `/cmd_vel`, `/astar_path`, `/robot_state`

### `robot_mapper`
Mapeamento incremental via grade de ocupação 2D usando ray casting (Bresenham). Não possui launch file próprio — é iniciado pelo `robot_controller.launch.py` do `robot_control`.

Tópicos consumidos: `/scan`, `/model/prm_robot/pose`  
Tópicos publicados: `/grid_map`

## Instalação

Este projeto depende do pacote `prm_2026`. Siga os passos abaixo:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clonar o pacote base da disciplina
git clone https://github.com/matheusbg8/prm_2026.git

# Clonar este repositório
git clone https://github.com/Hesctory/Movile-Robots-Programming.git

# Mover os pacotes da equipe para src/ e remover a pasta clonada
mv Movile-Robots-Programming/robot_control .
mv Movile-Robots-Programming/robot_mapper .
rm -rf Movile-Robots-Programming
```

O pacote `prm_2026` inclui um nó `robo_mapper.py` que conflita com o `robot_mapper` deste repositório. Delete-o antes de compilar:

```bash
rm ~/ros2_ws/src/prm_2026/prm_2026/robo_mapper.py
```

Compile e configure o ambiente:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select prm_2026 robot_control robot_mapper
source install/local_setup.bash
```

## Como executar

O projeto é iniciado em três terminais separados, nesta ordem:

**Terminal 1 — Gazebo (simulação)**
```bash
ros2 launch prm_2026 inicia_simulacao.launch.py
```

Parâmetro opcional `world`:
- `world:=arena_paredes.sdf` — arena com paredes como obstáculos (padrão)
- `world:=empty_arena.sdf` — arena sem obstáculos

**Terminal 2 — Robô (URDF, controladores, RViz, bridge)**
```bash
ros2 launch prm_2026 carrega_robo.launch.py
```

**Terminal 3 — Nossos nós (mapeamento + controle)**
```bash
ros2 launch robot_control robot_controller.launch.py
```
