# Trabalho de Robótica Móvel — ROS 2 Jazzy

Repositório do trabalho de robótica móvel da disciplina, implementado em **ROS 2 Jazzy** sobre **Ubuntu 24.04** (rodando em WSL2). O projeto consiste em quatro exercícios envolvendo navegação de um robô diferencial simulado em Gazebo Harmonic, equipado com sensor laser.

---

## Sumário

- [Visão geral do trabalho](#visão-geral-do-trabalho)
- [Estrutura do repositório](#estrutura-do-repositório)
- [Requisitos](#requisitos)
- [Instalação](#instalação)
- [Build do workspace](#build-do-workspace)
- [Como rodar](#como-rodar)
- [Validação e diagnóstico](#validação-e-diagnóstico)
- [Pacotes implementados](#pacotes-implementados)
- [Roadmap](#roadmap)
- [Troubleshooting](#troubleshooting)

---

## Visão geral do trabalho

O trabalho consiste em quatro exercícios que serão implementados em pacotes ROS 2 separados, todos usando o mesmo robô diferencial simulado:

1. **Tangent Bug** — navegação reativa entre dois pontos com detecção de "sem caminho" em tempo finito.
2. **Curva paramétrica** — controlador que faz o robô convergir e circular eternamente uma curva plana (ex: lemniscata).
3. **Campo potencial** — atrativo + repulsivo usando o laser para navegação entre dois pontos.
4. **Multi-robô** — composição dos exercícios 2 e 3 com vários robôs circulando a mesma curva, evitando obstáculos e colisões mútuas.

A ordem de implementação **não segue** a numeração do enunciado — a sequência adotada é 2 → 3 → 1 → 4, porque cada exercício reaproveita código do anterior.

---

## Estrutura do repositório

```
ros2_ws/src/                   ← raiz do repositório Git
├── README.md                  ← este arquivo
├── .gitignore
│
├── docs/                      ← relatório e documentação dos exercícios
│   ├── relatorio.md
│   ├── ex1_tangent_bug.md
│   ├── ex2_curva_parametrica.md
│   ├── ex3_potencial.md
│   ├── ex4_multi_robo.md
│   └── figuras/
│
├── nav_bringup/               ← [IMPLEMENTADO] simulação, robô, mundos, launches
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── urdf/
│   │   └── diff_robot.urdf.xacro
│   ├── worlds/
│   │   ├── empty.world
│   │   ├── obstacles_simple.world
│   │   └── obstacles_multi.world      (a criar)
│   ├── config/
│   │   ├── bridge_params.yaml
│   │   └── rviz/
│   │       └── nav.rviz
│   └── launch/
│       ├── sim_empty.launch.py
│       ├── sim_obstacles.launch.py
│       └── sim_multi_robot.launch.py  (a criar)
│
├── nav_common/                ← [PENDENTE] utilitários compartilhados
├── nav_msgs_custom/           ← [PENDENTE] interfaces de action
├── nav_tangent_bug/           ← [PENDENTE] Exercício 1
├── nav_parametric_curve/      ← [PENDENTE] Exercício 2
├── nav_potential_field/       ← [PENDENTE] Exercício 3
└── nav_multi_robot/           ← [PENDENTE] Exercício 4
```

---

## Requisitos

### Sistema

- **Ubuntu 24.04 LTS** (Noble Numbat)
- **WSL2** (se rodando no Windows) — Windows 11 recomendado pelo suporte nativo a GUI via WSLg
- **ROS 2 Jazzy Jalisco**
- **Gazebo Harmonic** (vem com `ros-gz` no Jazzy)

### Pacotes ROS 2 necessários

Todos instalados via `apt`:

- `ros-jazzy-desktop` — instalação completa do ROS 2 (inclui RViz, demos)
- `ros-dev-tools` — colcon, rosdep, vcs, ament tools
- `ros-jazzy-ros-gz` — integração ROS ↔ Gazebo Harmonic
- `ros-jazzy-ros-gz-bridge` — bridge de tópicos
- `ros-jazzy-ros-gz-sim` — launchers do Gazebo
- `ros-jazzy-xacro` — pré-processador de URDF
- `ros-jazzy-robot-state-publisher` — publica TF do URDF
- `ros-jazzy-joint-state-publisher`
- `ros-jazzy-rviz2` — visualizador 3D

### Ferramentas auxiliares

- `liburdfdom-tools` — `check_urdf` para validar URDFs
- `tree` — visualização de estrutura de pastas

---

## Instalação

### 1. WSL2 e Ubuntu 24.04

No PowerShell do Windows (como administrador):

```powershell
wsl --install -d Ubuntu-24.04
wsl --set-default-version 2
```

Abra o Ubuntu, crie usuário e atualize:

```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Locale

```bash
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3. Repositório do ROS 2

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
```

### 4. ROS 2 Jazzy + ferramentas

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
    liburdfdom-tools \
    tree
```

### 5. Source automático no `.bashrc`

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. Clonar este repositório

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <URL_DO_REPO> .
```

---

## Build do workspace

### Build completo (primeira vez)

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

A flag `--symlink-install` cria links simbólicos para arquivos Python, launches, configs e URDFs em vez de copiar — então edições nesses arquivos refletem imediatamente sem precisar recompilar.

### Build de um pacote específico

```bash
colcon build --packages-select nav_bringup --symlink-install
source install/setup.bash
```

### Limpar build

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

---

## Como rodar

### Simulação com mundo vazio

```bash
ros2 launch nav_bringup sim_empty.launch.py
```

Sobe:
- Gazebo Harmonic com o `empty.world`
- Robô diferencial spawnado em (0, 0, 0.1)
- `robot_state_publisher` publicando o TF do URDF
- Bridge ROS ↔ Gazebo
- RViz com a configuração `nav.rviz`

### Simulação com obstáculos

```bash
ros2 launch nav_bringup sim_obstacles.launch.py
```

Mesma configuração, mas carrega o `obstacles_simple.world`, que contém:
- Arena 20×20 m delimitada por paredes
- Três cilindros verdes
- Duas caixas marrons
- Uma armadilha em U cinza (para testar mínimo local de campo potencial)

### Mover o robô manualmente

Em outro terminal:

```bash
# Andar para frente
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Girar no lugar
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Parar
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## Validação e diagnóstico

### Verificar tópicos ROS

```bash
ros2 topic list
```

Deve listar pelo menos:
```
/clock
/cmd_vel
/joint_states
/odom
/robot_description
/scan
/tf
/tf_static
```

### Verificar leituras do laser

```bash
ros2 topic hz /scan          # taxa esperada: ~10 Hz
ros2 topic echo /scan --once # imprime um LaserScan completo
```

### Verificar odometria

```bash
ros2 topic echo /odom --once
```

### Verificar árvore TF

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

Deve imprimir translação e rotação a cada segundo. Se der "Could not find a connection", a árvore está quebrada — ver [Troubleshooting](#troubleshooting).

### Inspecionar tópicos do Gazebo

```bash
gz topic -l                  # lista tópicos do lado Gazebo
gz topic -i -t /scan         # info de um tópico
gz model --list              # lista modelos no mundo
```

---

## Pacotes implementados

### `nav_bringup`

Pacote `ament_cmake` que centraliza toda a infraestrutura de simulação. Não contém algoritmos — apenas descrição do robô, mundos e launches que sobem o ambiente.

**URDF (`urdf/diff_robot.urdf.xacro`)** — robô diferencial com:
- Chassis 40×30×15 cm, massa 5 kg
- Duas rodas ativas (raio 8 cm), separação 34 cm
- Caster traseiro sem atrito (esfera de apoio)
- Sensor LIDAR 2D no topo: 360 amostras em 360°, alcance 0.12 m a 8 m, taxa 10 Hz
- Macros xacro para inércias de caixa, cilindro e esfera (calculadas a partir de massa e dimensões)

**Plugins Gazebo embutidos no URDF:**
- `DiffDrive` — recebe `/cmd_vel` e move as rodas
- `OdometryPublisher` — publica `odom → base_footprint` no `/tf`
- `JointStatePublisher` — publica `/joint_states`
- Sensor `gpu_lidar` — publica `/scan`

**Mundos (`worlds/`):**
- `empty.world` — plano vazio para o exercício 2 (curva paramétrica)
- `obstacles_simple.world` — arena 20×20 com cilindros, caixas e obstáculo em U para os exercícios 1 e 3

**Bridge (`config/bridge_params.yaml`)** — mapeia tópicos entre Gazebo e ROS:

| Tópico | Direção | Tipo ROS |
|---|---|---|
| `/cmd_vel` | ROS → GZ | `geometry_msgs/Twist` |
| `/odom` | GZ → ROS | `nav_msgs/Odometry` |
| `/tf` | GZ → ROS | `tf2_msgs/TFMessage` |
| `/joint_states` | GZ → ROS | `sensor_msgs/JointState` |
| `/scan` | GZ → ROS | `sensor_msgs/LaserScan` |
| `/clock` | GZ → ROS | `rosgraph_msgs/Clock` |

**RViz (`config/rviz/nav.rviz`)** — pré-configurado com displays:
- Grid
- RobotModel (lê `/robot_description`)
- TF (eixos de todos os links)
- LaserScan em `/scan` (Best Effort QoS)
- Odometry em `/odom`
- Fixed Frame: `odom`

**Launches (`launch/`):**
- `sim_empty.launch.py` — launch base, parametrizável via argumento `world`
- `sim_obstacles.launch.py` — chama o `sim_empty.launch.py` passando `obstacles_simple.world`

---

## Roadmap

| Fase | Pacote | Status | Descrição |
|---|---|---|---|
| 0 | — | ✔ | Estrutura do repositório, Git, .gitignore |
| 1 | `nav_bringup` | ✔ | Simulação, URDF, mundos, launches, RViz |
| 2 | `nav_common` | ⏳ | Utilitários: geometria, laser, controle diferencial |
| 3 | `nav_parametric_curve` | ⏳ | Exercício 2 — curva paramétrica |
| 4 | `nav_potential_field` | ⏳ | Exercício 3 — campo potencial |
| 5 | `nav_msgs_custom` | ⏳ | Action `NavigateToGoal` |
| 6 | `nav_tangent_bug` | ⏳ | Exercício 1 — Tangent Bug |
| 7 | `nav_multi_robot` | ⏳ | Exercício 4 — multi-robô |
| 8 | `docs/` | ⏳ | Relatório final, vídeos, gráficos |

---

## Troubleshooting

### Erro `Unable to parse the value of parameter robot_description as yaml`

**Causa:** o `launch_ros` tenta interpretar o resultado do `xacro` como YAML antes de passar como string.
**Solução:** o launch já está corrigido usando `ParameterValue(..., value_type=str)` ao redor do `Command(['xacro ', urdf_file])`.

### Robô aparece no Gazebo mas não no RViz

**Causa mais comum:** a árvore TF está incompleta — falta a transformação `odom → base_footprint`, que precisa vir do plugin DiffDrive ou OdometryPublisher do Gazebo via bridge.

**Diagnóstico:**

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

Se der "Could not find a connection", o problema é esse.

**Solução:** o URDF inclui o plugin `OdometryPublisher` justamente para garantir a publicação dessa transformação. Certifique-se que o bloco está presente em `diff_robot.urdf.xacro`:

```xml
<gazebo>
  <plugin filename="gz-sim-odometry-publisher-system"
          name="gz::sim::systems::OdometryPublisher">
    <odom_frame>odom</odom_frame>
    <robot_base_frame>base_footprint</robot_base_frame>
    <odom_publish_frequency>50</odom_publish_frequency>
    <tf_topic>tf</tf_topic>
  </plugin>
</gazebo>
```

### `/scan` não aparece em `ros2 topic list`

**Causa:** o plugin `gz::sim::systems::Sensors` não está no arquivo de mundo.
**Solução:** verifique que o mundo (`empty.world`, `obstacles_simple.world`) inclui o bloco:

```xml
<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

### `/scan` aparece mas RViz não mostra os pontos

**Causa:** mismatch de QoS — Gazebo publica em "Best Effort", RViz por padrão escuta "Reliable".
**Solução:** no display LaserScan do RViz, mudar `Reliability Policy` para `Best Effort`. O `nav.rviz` já tem essa configuração.

### Gazebo abre tela preta no WSL

**Causa:** problema de aceleração gráfica.
**Solução:**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch nav_bringup sim_empty.launch.py
```
Mais lento, mas funciona.

### Processos órfãos do Gazebo

Se o Gazebo travar e fechar mal, processos podem ficar pendurados e impedir o próximo launch:

```bash
pkill -9 -f "gz sim"
pkill -9 -f "parameter_bridge"
pkill -9 -f "rviz2"
pkill -9 -f "robot_state_publisher"
```

### Avisos `RTPS_TRANSPORT_SHM Error`

Inofensivos no WSL — relacionados a memória compartilhada do Fast DDS. Podem ser ignorados.

---

## Referências

- [Documentação oficial ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [ros_gz tutorials](https://github.com/gazebosim/ros_gz)
- [URDF Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)

---

## Autor

Gabriel Vaz Cançado Ferreira - gabrielvazcancadoferreira@gmail.com
Philip Ribeiro Costa
