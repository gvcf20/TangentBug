# Trabalho de Robótica Móvel — Navegação Reativa com ROS 2

Repositório do trabalho de robótica móvel implementado em **ROS 2 Jazzy** com **Gazebo Harmonic**, usando o **TurtleBot3 Burger** como plataforma de simulação. O projeto implementa quatro estratégias de navegação para robôs com acionamento diferencial equipados com sensor laser.

---

## Sumário

- [Exercícios](#exercícios)
- [Estrutura do Repositório](#estrutura-do-repositório)
- [Arquitetura ROS 2](#arquitetura-ros-2)
- [Requisitos](#requisitos)
- [Instalação](#instalação)
- [Build](#build)
- [Executando os Exercícios](#executando-os-exercícios)
- [Descrição dos Pacotes](#descrição-dos-pacotes)
- [Parâmetros Configuráveis](#parâmetros-configuráveis)
- [Troubleshooting](#troubleshooting)
- [Autor](#autor)

---

## Exercícios

### Exercício 1 — Tangent Bug (`nav_tangent_bug`)

Algoritmo de navegação reativa tipo Tangent Bug que navega um robô diferencial entre duas posições quaisquer, escolhidas pelo usuário em tempo de execução, sem colidir com obstáculos. Se não houver caminho entre as posições, o robô informa ao usuário em tempo finito.

O algoritmo opera com duas modalidades: **Motion-to-Goal** (vai em linha reta à meta) e **Boundary-Following** (contorna obstáculos quando o caminho direto está bloqueado). A transição entre estados é governada pelas heurísticas d_reach e d_followed.

### Exercício 2 — Curva Paramétrica (`nav_parametric_curve`)

Controlador baseado em campo vetorial que faz o robô convergir e circular eternamente uma lemniscata de Bernoulli. O campo combina um componente normal (convergência) com um componente tangente (circulação), ponderados por tanh para transição suave.

### Exercício 3 — Campo Potencial (`nav_potential_field`)

Navegação por potencial atrativo (meta) + repulsivo (obstáculos via laser) entre duas posições quaisquer num ambiente com obstáculos. Inclui detecção de mínimo local com wall-following orientado à meta como estratégia de escape.

### Exercício 4 — Multi-Robô (`nav_multi_robot`)

Composição dos exercícios 2 e 3: dois ou mais robôs navegam para convergir e circular a mesma curva paramétrica num ambiente com obstáculos estáticos. Os robôs usam funções de potencial para evitar obstáculos e colisões entre si, com cada robô tendo acesso à posição dos demais.

---

## Estrutura do Repositório

```
ros2_ws/src/                          ← raiz do repositório Git
├── README.md
├── .gitignore
│
├── docs/                             ← relatório e documentação
│   ├── relatorio.md
│   ├── ex1_tangent_bug.md
│   ├── ex2_curva_parametrica.md
│   ├── ex3_potencial.md
│   ├── ex4_multi_robo.md
│   └── figuras/
│
├── nav_bringup/                      ← simulação, robô, mundos, launches
│   ├── urdf/
│   │   ├── turtlebot3_burger.urdf    ← TurtleBot3 + plugins Gazebo
│   │   └── diff_robot.urdf.xacro     ← robô customizado (alternativo)
│   ├── worlds/
│   │   ├── empty.world
│   │   ├── obstacles_simple.world
│   │   └── obstacles_multi.world
│   ├── config/
│   │   ├── bridge_params.yaml
│   │   └── rviz/nav.rviz
│   └── launch/
│       ├── sim_empty.launch.py
│       ├── sim_obstacles.launch.py
│       └── sim_multi_robot.launch.py
│
├── nav_common/                       ← utilitários compartilhados
│   ├── nav_common/
│   │   ├── geometry.py               ← wrap_to_pi, yaw_from_quaternion, distance
│   │   ├── diff_drive.py             ← force_to_twist, saturate_twist
│   │   ├── laser_utils.py            ← scan_to_points, find_discontinuities
│   │   ├── tf_helpers.py             ← TFHelper (wrapper tf2)
│   │   └── plotting.py              ← TrajectoryLogger (CSV)
│   └── test/
│       ├── test_geometry.py          ← 17 testes
│       └── test_laser_utils.py       ← 10 testes
│
├── nav_msgs_custom/                  ← interfaces de action
│   └── action/
│       └── NavigateToGoal.action
│
├── nav_tangent_bug/                  ← Exercício 1
│   ├── nav_tangent_bug/
│   │   ├── states.py                 ← TBState enum
│   │   ├── heuristic.py              ← d_reach, find_best_tangent_point
│   │   ├── tangent_bug_node.py       ← servidor de action
│   │   └── client_node.py            ← cliente para enviar metas
│   ├── config/tangent_bug.yaml
│   └── launch/tangent_bug.launch.py
│
├── nav_parametric_curve/             ← Exercício 2
│   ├── nav_parametric_curve/
│   │   ├── curves.py                 ← Lemniscate, Cardioid
│   │   ├── vector_field.py           ← compute_field (normal + tangente)
│   │   └── curve_follower_node.py    ← nó ROS
│   ├── config/curve_params.yaml
│   └── launch/curve_follower.launch.py
│
├── nav_potential_field/              ← Exercício 3
│   ├── nav_potential_field/
│   │   ├── attractive.py             ← compute_attractive (parabólico + cônico)
│   │   ├── repulsive.py              ← compute_repulsive (laser + pontos)
│   │   └── potential_field_node.py   ← nó ROS com wall-follow escape
│   ├── config/potential_params.yaml
│   └── launch/potential_field.launch.py
│
└── nav_multi_robot/                  ← Exercício 4
    ├── nav_multi_robot/
    │   ├── composition.py            ← F = α·F_curva + β·F_rep_obs + γ·F_rep_rob
    │   └── multi_robot_node.py       ← nó por robô
    ├── config/multi_robot_params.yaml
    └── launch/multi_robot.launch.py
```

---

## Arquitetura ROS 2

### Fluxo de dados (todos os exercícios)

```
Gazebo Harmonic
    │
    ├── publica /scan (LaserScan, via gpu_lidar)
    ├── publica /odom (Odometry, via OdometryPublisher)
    ├── publica /tf (TFMessage, via OdometryPublisher)
    ├── escuta  /cmd_vel (Twist, via DiffDrive)
    │
    └── ros_gz_bridge traduz entre formatos Gazebo ↔ ROS
            │
            ▼
    Nó controlador (exercício específico)
        ├── subscreve /odom → extrai (x, y, yaw)
        ├── subscreve /scan → lê obstáculos
        ├── calcula campo vetorial / heurística
        ├── converte para (v, ω) via force_to_twist()
        └── publica /cmd_vel
```

### Configuração de odometria (crítica)

O URDF do TurtleBot3 usa **dois plugins separados** no Gazebo:

- **DiffDrive**: recebe `/cmd_vel` e move as rodas. **Não publica odometria nem TF** — apenas controle motor.
- **OdometryPublisher**: única fonte de `/odom` e `/tf`, usando a posição real do Gazebo. Zero drift.

Essa separação é essencial para evitar divergência entre odometria e posição real, que causaria comportamento errático nos algoritmos de navegação.

### Comunicação no exercício 4 (multi-robô)

```
           robot_0                         robot_1
    ┌─────────────────┐            ┌─────────────────┐
    │ /robot_0/odom   │◄──────────►│ /robot_1/odom   │
    │ /robot_0/scan   │            │ /robot_1/scan   │
    │ /robot_0/cmd_vel│            │ /robot_1/cmd_vel│
    │                 │            │                 │
    │ Lê odom do      │            │ Lê odom do      │
    │ robot_1 para    │            │ robot_0 para    │
    │ repulsão mútua  │            │ repulsão mútua  │
    └─────────────────┘            └─────────────────┘
```

---

## Requisitos

### Sistema

- Ubuntu 24.04 LTS (Noble Numbat)
- WSL2 (se rodando no Windows) — Windows 11 recomendado pelo WSLg
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic

### Pacotes ROS 2

```
ros-jazzy-desktop
ros-dev-tools
ros-jazzy-ros-gz
ros-jazzy-ros-gz-bridge
ros-jazzy-ros-gz-sim
ros-jazzy-xacro
ros-jazzy-robot-state-publisher
ros-jazzy-joint-state-publisher
ros-jazzy-rviz2
ros-jazzy-turtlebot3-description
```

### Pacotes Python

```
transforms3d
```

---

## Instalação

### 1. ROS 2 Jazzy

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
```

### 2. Pacotes

```bash
sudo apt install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-turtlebot3-description \
    liburdfdom-tools

pip install transforms3d --break-system-packages
```

### 3. Source automático

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Clonar o repositório

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <URL_DO_REPO> .
```

---

## Build

### Build completo (primeira vez)

```bash
cd ~/ros2_ws

# nav_msgs_custom precisa de build normal (interfaces geram código compilado)
colcon build --packages-select nav_msgs_custom

# Todo o resto com symlink-install (edições refletem sem recompilar)
colcon build --packages-ignore nav_msgs_custom --symlink-install

source install/setup.bash
```

### Build de um pacote específico

```bash
colcon build --packages-select nav_tangent_bug --symlink-install
source install/setup.bash
```

### Limpar e rebuildar

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select nav_msgs_custom
colcon build --packages-ignore nav_msgs_custom --symlink-install
source install/setup.bash
```

### Rodar testes unitários

```bash
colcon test --packages-select nav_common
colcon test-result --verbose
```

---

## Executando os Exercícios

### Exercício 2 — Curva Paramétrica

```bash
ros2 launch nav_parametric_curve curve_follower.launch.py
```

O robô inicia em (0,0), converge para a lemniscata e circula indefinidamente. No RViz, adicione um display Marker no tópico `/curve_marker` para ver a curva alvo em verde.

Parâmetros ajustáveis em tempo real:
```bash
ros2 param set /curve_follower v_max 0.3
ros2 param set /curve_follower k_normal 2.0
```

### Exercício 3 — Campo Potencial

```bash
ros2 launch nav_potential_field potential_field.launch.py
```

Envie metas de duas formas:

Via **RViz** (recomendado): clique no botão "2D Goal Pose" na toolbar e clique no ponto desejado do mapa.

Via **terminal**:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 4.0, y: 0.0}}}"
```

No RViz, adicione Marker em `/potential_markers` para ver a meta como esfera verde.

### Exercício 1 — Tangent Bug

```bash
# Terminal 1: launch
ros2 launch nav_tangent_bug tangent_bug.launch.py

# Terminal 2: enviar meta
ros2 run nav_tangent_bug tangent_bug_client -- --x 5.0 --y 3.0
```

O cliente mostra feedback contínuo com o estado atual, distância à meta, e heurísticas d_reach / d_followed:

```
[motion_to_goal]      d_goal=5.00 d_reach=5.00 d_followed=inf
[boundary_following]  d_goal=3.50 d_reach=3.50 d_followed=3.50
[motion_to_goal]      d_goal=1.20 d_reach=1.20 d_followed=3.50
SUCESSO: goal_reached
```

Se não houver caminho:
```
FALHA: no_path_found
```

### Exercício 4 — Multi-Robô

```bash
# 2 robôs (padrão)
ros2 launch nav_multi_robot multi_robot.launch.py

# 3 robôs
ros2 launch nav_multi_robot multi_robot.launch.py n_robots:=3
```

Os robôs iniciam em posições equidistantes num raio de 3 m, convergem para a lemniscata, e circulam evitando obstáculos e uns aos outros.

### Mover o robô manualmente (para debug)

```bash
# Andar para frente
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# Girar
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Parar
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## Descrição dos Pacotes

### `nav_bringup`

Infraestrutura de simulação. Não contém algoritmos — apenas o robô, mundos e launches.

**Robô**: TurtleBot3 Burger com plugins Gazebo Harmonic:
- DiffDrive (só cmd_vel, sem odom)
- OdometryPublisher (única fonte de /odom e /tf, posição real)
- JointStatePublisher
- GPU Lidar (360 amostras, 5 Hz, alcance 0.12–3.5 m)

**Mundos**:
- `empty.world` — plano vazio (exercício 2)
- `obstacles_simple.world` — arena 20×20 m com cilindros, caixas e obstáculo em U (exercícios 1 e 3)
- `obstacles_multi.world` — obstáculos posicionados fora da lemniscata (exercício 4)

**Bridge** (`bridge_params.yaml`):

| Tópico | Direção | Tipo ROS |
|---|---|---|
| `/cmd_vel` | ROS → GZ | `geometry_msgs/Twist` |
| `/odom` | GZ → ROS | `nav_msgs/Odometry` |
| `/tf` | GZ → ROS | `tf2_msgs/TFMessage` |
| `/joint_states` | GZ → ROS | `sensor_msgs/JointState` |
| `/scan` | GZ → ROS | `sensor_msgs/LaserScan` |
| `/clock` | GZ → ROS | `rosgraph_msgs/Clock` |

### `nav_common`

Utilitários reutilizados por todos os exercícios:

- **`geometry.py`**: `wrap_to_pi`, `yaw_from_quaternion` (via transforms3d), `distance`, `angle_to_target`, `angle_diff`
- **`diff_drive.py`**: `force_to_twist` — converte vetor de força (Fx,Fy) no frame do mundo para Twist (v,ω) com saturação e alinhamento por cos(erro)
- **`laser_utils.py`**: `scan_to_points`, `scan_to_polar`, `find_discontinuities` (para Tangent Bug), `closest_obstacle`, `is_path_clear`
- **`tf_helpers.py`**: wrapper tf2 para obter pose via `TFHelper.get_pose()`
- **`plotting.py`**: `TrajectoryLogger` para salvar trajetórias em CSV

27 testes unitários cobrindo geometria e laser.

### `nav_msgs_custom`

Define a action `NavigateToGoal` usada pelo Tangent Bug:

```
# Goal
geometry_msgs/Point target
---
# Result
bool success
string message       # "goal_reached" | "no_path_found" | "cancelled"
---
# Feedback
float32 distance_to_goal
string current_state  # "motion_to_goal" | "boundary_following"
float32 d_reach
float32 d_followed
```

### `nav_parametric_curve`

**`curves.py`**: classes `Lemniscate` e `Cardioid` com métodos `evaluate(t)`, `tangent(t)`, `find_closest_t(x,y)`.

**`vector_field.py`**: campo vetorial que combina componente normal (tanh para convergência suave) e tangente (circulação), com transição automática baseada na distância à curva.

**`curve_follower_node.py`**: nó que subscreve `/odom`, calcula campo, converte para twist, publica `/cmd_vel` a 20 Hz. Publica a curva alvo como Marker no RViz.

### `nav_potential_field`

**`attractive.py`**: potencial atrativo com dois regimes — parabólico (perto, desacelera) e cônico (longe, força constante).

**`repulsive.py`**: potencial repulsivo calculado a partir do LaserScan com transformação de frame laser→mundo. Inclui `compute_repulsive_from_points` para repulsão entre robôs no exercício 4.

**`potential_field_node.py`**: recebe metas via `/goal_pose` (compatível com botão "2D Goal Pose" do RViz). Inclui detecção de mínimo local com wall-following orientado à meta.

**Limitação documentada**: mínimos locais em obstáculos côncavos (formato U). Mitigação via wall-follow com alternância de lado, eficaz para obstáculos convexos.

### `nav_tangent_bug`

**`states.py`**: enum `TBState` com `MOTION_TO_GOAL`, `BOUNDARY_FOLLOWING`, `GOAL_REACHED`, `NO_PATH`.

**`heuristic.py`**: `compute_d_reach` (menor distância à meta via reta livre) e `find_best_tangent_point` (melhor ponto de descontinuidade para contornar).

**`tangent_bug_node.py`**: servidor de action `NavigateToGoal` com:
- Máquina de estados completa
- Escolha de lado de contorno via produto vetorial
- Detecção de estagnação no boundary-following (5s timeout)
- Detecção de travamento físico (3s sem movimento → recua e gira)
- Detecção de volta completa (no_path)

**`client_node.py`**: cliente para enviar metas via linha de comando.

### `nav_multi_robot`

**`composition.py`**: `compute_composed_field` que soma ponderadamente:
- α · F_curva (convergir e circular a lemniscata)
- β · F_rep_obstáculos (repulsão via laser)
- γ · F_rep_robôs (repulsão via posições conhecidas)

**`multi_robot_node.py`**: nó por robô que subscreve seu próprio `/robotN/odom` e `/robotN/scan`, mais o `/robotM/odom` dos outros robôs para repulsão mútua.

---

## Parâmetros Configuráveis

Todos os parâmetros são carregados via YAML e podem ser ajustados sem recompilar.

### Exercício 2 (`config/curve_params.yaml`)

| Parâmetro | Default | Descrição |
|---|---|---|
| `curve_type` | `lemniscate` | Tipo de curva (`lemniscate` ou `cardioid`) |
| `curve_scale` | `2.0` | Escala da curva (metade da extensão em x) |
| `k_normal` | `1.5` | Ganho de convergência |
| `k_tangent` | `1.0` | Ganho de circulação |
| `v_max` | `0.22` | Velocidade linear máxima (m/s) |

### Exercício 3 (`config/potential_params.yaml`)

| Parâmetro | Default | Descrição |
|---|---|---|
| `k_att` | `1.0` | Ganho atrativo |
| `k_rep` | `1.0` | Ganho repulsivo |
| `d0` | `1.2` | Distância de influência dos obstáculos (m) |
| `goal_tolerance` | `0.15` | Distância para considerar "chegou" (m) |
| `wall_follow_distance` | `0.35` | Distância da parede no wall-follow (m) |

### Exercício 1 (`config/tangent_bug.yaml`)

| Parâmetro | Default | Descrição |
|---|---|---|
| `safe_distance` | `0.50` | Distância para considerar caminho bloqueado (m) |
| `wall_follow_distance` | `0.45` | Distância da parede no boundary-following (m) |
| `goal_tolerance` | `0.10` | Tolerância de chegada (m) |
| `bf_stagnation_timeout` | `5.0` | Timeout de estagnação no BF (s) |
| `physical_stuck_timeout` | `3.0` | Timeout de travamento físico (s) |
| `loop_closure_min_travel` | `1.5` | Mínimo percorrido antes de checar volta completa (m) |

### Exercício 4 (`config/multi_robot_params.yaml`)

| Parâmetro | Default | Descrição |
|---|---|---|
| `alpha` | `1.0` | Peso do campo da curva |
| `beta` | `1.0` | Peso da repulsão de obstáculos |
| `gamma` | `1.5` | Peso da repulsão entre robôs |
| `k_rep_robot` | `2.0` | Ganho de repulsão inter-robô |
| `d0_robot` | `1.5` | Distância de influência inter-robô (m) |
| `n_robots` | `2` | Número de robôs |

---

## Troubleshooting

### Robô não aparece no RViz

**Causa**: árvore TF incompleta — falta a transformação `odom → base_footprint`.
**Verificar**: `ros2 run tf2_ros tf2_echo odom base_footprint`
**Solução**: confirmar que o plugin `OdometryPublisher` está no URDF com `<tf_topic>tf</tf_topic>`.

### Odometria diverge da posição real do Gazebo

**Causa**: dois plugins publicando odometria (DiffDrive + OdometryPublisher) com fontes diferentes.
**Solução**: o DiffDrive deve ter **apenas** `<left_joint>`, `<right_joint>`, `<wheel_separation>`, `<wheel_radius>` e `<topic>`. Sem `<odom_topic>` nem `<tf_topic>`. O OdometryPublisher é a única fonte.

**Verificar**:
```bash
gz model -m diff_robot --pose       # posição real
ros2 topic echo /odom --once --field pose.pose.position  # odometria
```
Devem ser praticamente iguais.

### `/scan` não aparece ou não mostra dados

**Causa**: plugin `Sensors` ausente no arquivo de mundo.
**Solução**: verificar que o mundo inclui:
```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

### RViz não mostra pontos do laser

**Causa**: mismatch de QoS — Gazebo publica em Best Effort.
**Solução**: no display LaserScan do RViz, mudar Reliability Policy para Best Effort.

### `colcon build` falha no `nav_msgs_custom` com symlink-install

**Causa**: pacotes de interface não suportam `--symlink-install`.
**Solução**:
```bash
colcon build --packages-select nav_msgs_custom          # sem symlink
colcon build --packages-ignore nav_msgs_custom --symlink-install  # resto com symlink
```

### Erro `Unable to parse the value of parameter robot_description as yaml`

**Causa**: o launch_ros tenta interpretar o URDF como YAML.
**Solução**: usar `ParameterValue(..., value_type=str)` ao redor do conteúdo do URDF no launch file.

### Robô fica travado contra obstáculo (Tangent Bug)

**Causa**: o wall-follow perdeu contato com o obstáculo (cilindros pequenos).
**Solução**: o nó inclui detecção de estagnação (5s) e travamento físico (3s). Quando detecta, recua, gira e tenta novamente.

### Mínimo local no campo potencial (obstáculo em U)

**Causa**: limitação fundamental do método — atrativo e repulsivo se cancelam.
**Resultado esperado**: o robô pode ficar preso. Documentado no relatório como limitação conhecida, com comparação ao Tangent Bug que resolve o cenário.

### Gazebo abre tela preta no WSL

**Solução**: `export LIBGL_ALWAYS_SOFTWARE=1` antes do launch.

### Processos órfãos do Gazebo

Se o Gazebo travar ao fechar:
```bash
pkill -9 -f "gz sim"
pkill -9 -f "parameter_bridge"
pkill -9 -f "rviz2"
pkill -9 -f "robot_state_publisher"
```

### Warnings `RTPS_TRANSPORT_SHM Error`

Inofensivos no WSL — relacionados a memória compartilhada do Fast DDS. Podem ser ignorados.

---

## Validação Rápida

Script para verificar que tudo está funcionando:

```bash
source ~/ros2_ws/install/setup.bash

# 1. Tópicos disponíveis
ros2 topic list

# 2. Laser publicando
ros2 topic hz /scan

# 3. Odometria consistente
gz model -m diff_robot --pose
ros2 topic echo /odom --once --field pose.pose.position

# 4. TF conectado
ros2 run tf2_ros tf2_echo odom base_footprint

# 5. Robô se move
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# 6. Interfaces customizadas
ros2 interface show nav_msgs_custom/action/NavigateToGoal

# 7. Testes unitários
colcon test --packages-select nav_common && colcon test-result --verbose
```

---

## Referências

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [ros_gz Integration](https://github.com/gazebosim/ros_gz)
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- Choset, H. et al. *Principles of Robot Motion: Theory, Algorithms, and Implementations*. MIT Press, 2005. (Tangent Bug, Potential Fields, Bug Algorithms)

---

## Autor

Gabriel Vaz Cançado Ferreira — Engenharia Elétrica, UFMG
Phillip Ribeiro Costa — Engenharia Elétrica, UFMG
Trabalho de Planejamento de Movimento de Robôs