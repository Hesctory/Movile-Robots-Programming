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
Controle autônomo do robô com máquina de estados completa de captura e retorno da bandeira:

- **EXPLORING** — spin inicial, avanço com desvio de obstáculos por histerese e perseguição do último rumo conhecido da bandeira
- **FLAG_DETECTED** — confirmação da detecção via câmera com Schmitt-trigger e janela de graça para debounce; creep lento em direção à bandeira enquanto acumula votos de confirmação
- **NAVIGATING_TO_FLAG** — planejamento de caminho A* até a bandeira, com guarda para não circunavegar o próprio alvo quando o obstáculo à frente é o mastro
- **POSITION_TO_COLLECT** — aproximação fina por visual servoing; abre a garra na entrada do estado
- **GRABBING** — para o robô e fecha a garra em torno do mastro (open-loop temporizado, ~1,5 s)
- **LIFTING** — levanta o braço com a garra fechada (~3 s)
- **RETURNING_TO_BASE** — navega de volta à posição inicial registrada no arranque, usando A* com re-planejamento periódico; retornos LiDAR do próprio cargo são mascarados para não confundir o planejador

Recursos adicionais:
- **Recuperação de tombamento** — detecta inclinação excessiva via IMU, recua, registra células de risco com penalidade no custo A* e avança para sair; suprimida durante GRABBING e LIFTING
- **Re-aquisição após perda da bandeira** — spin de busca de até ~6 s antes de voltar a EXPLORING

Tópicos consumidos: `/scan`, `/robot_cam/labels_map`, `/model/prm_robot/pose`, `/grid_map`, `/imu`  
Tópicos publicados: `/cmd_vel`, `/astar_path`, `/robot_state`, `/gripper_controller/commands`

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
