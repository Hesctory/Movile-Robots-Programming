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
- **EXPLORING** — navegação aleatória com desvio de obstáculos
- **FLAG_DETECTED** — confirmação de detecção da bandeira pela câmera
- **NAVIGATING_TO_FLAG** — planejamento de caminho A* até a bandeira
- **POSITION_TO_COLLECT** — aproximação fina para coleta

Tópicos consumidos: `/scan`, `/robot_cam/labels_map`, `/model/prm_robot/pose`, `/grid_map`, `/imu`  
Tópicos publicados: `/cmd_vel`, `/astar_path`

### `robot_mapper`
Mapeamento incremental via grade de ocupação 2D usando ray casting (Bresenham).

Tópicos consumidos: `/scan`, `/model/prm_robot/pose`  
Tópicos publicados: `/grid_map`

## Como executar

O projeto é iniciado em três terminais separados, nesta ordem:

**Terminal 1 — Gazebo (simulação)**
```bash
ros2 launch prm_2026 inicia_simulacao.launch.py
```

**Terminal 2 — Robô (URDF, controladores, RViz, bridge)**
```bash
ros2 launch prm_2026 carrega_robo.launch.py
```

**Terminal 3 — Nossos nós (mapeamento + controle)**
```bash
ros2 launch robot_control robot_controller.launch.py
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select robot_control robot_mapper
source install/setup.bash
```
