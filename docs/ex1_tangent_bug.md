# Exercício 1 — Tangent Bug

## Objetivo

Implementar um algoritmo tipo Tangent Bug que navegue um robô com acionamento diferencial simulado em um ambiente com obstáculos. O robô deve se mover entre duas posições quaisquer, escolhidas pelo usuário em tempo de execução, sem colidir com os obstáculos. Caso não haja caminho entre as posições escolhidas, o robô deve informar isto ao usuário em um tempo finito.

## Fundamentação Teórica

### O Algoritmo Tangent Bug

O Tangent Bug é uma variante dos algoritmos "Bug" que utiliza um sensor de alcance finito (laser) para navegar localmente com **garantia de completude**: se existe caminho, o algoritmo encontra; se não existe, detecta em tempo finito.

O algoritmo opera com duas modalidades principais:

**Motion-to-Goal (MTG):** O robô se move em linha reta na direção da meta. Permanece neste estado enquanto o caminho direto estiver livre (verificado pelo cone de feixes do laser na direção da meta). Quando um obstáculo bloqueia o caminho, transiciona para Boundary-Following.

**Boundary-Following (BF):** O robô segue o contorno do obstáculo, mantendo uma distância fixa da parede. Durante o contorno, monitora duas heurísticas para decidir quando sair:

- **d_reach**: menor distância à meta alcançável por uma reta livre a partir da posição atual, calculada usando os pontos visíveis do laser.
- **d_followed**: menor distância à meta observada ao longo do contorno atual.

### Condições de Transição

**BF → MTG:** Ocorre quando o caminho direto à meta fica livre (verificado por `is_path_clear`) ou quando d_reach < d_followed (encontrou atalho).

**BF → NO_PATH:** Ocorre quando o robô completa uma volta ao redor do obstáculo sem encontrar saída (distância ao ponto de entrada < limiar após percorrer distância mínima).

### Escolha do Lado de Contorno

O lado de contorno (esquerda ou direita) é escolhido pelo **produto vetorial** entre os vetores robô→obstáculo e robô→meta. O sinal do produto indica de qual lado da reta robô-obstáculo a meta está, garantindo que o robô contorne pelo lado mais curto.

### Detecção de Travamento

Dois mecanismos complementares:

- **Estagnação no BF** (5s): se d_goal não melhora durante o boundary-following, volta para MTG e tenta ir direto.
- **Travamento físico** (3s): se a posição do robô não muda em nenhum estado, recua 1s e gira 90° para se desobstruir.

## Implementação

### Pacote: `nav_tangent_bug`

```
nav_tangent_bug/
├── nav_tangent_bug/
│   ├── states.py              ← enum TBState (MOTION_TO_GOAL, BOUNDARY_FOLLOWING, GOAL_REACHED, NO_PATH)
│   ├── heuristic.py           ← compute_d_reach, find_best_tangent_point
│   ├── tangent_bug_node.py    ← servidor de action NavigateToGoal
│   └── client_node.py         ← cliente para enviar metas via terminal
├── config/tangent_bug.yaml    ← parâmetros configuráveis
└── launch/tangent_bug.launch.py
```

### Interface de Comunicação

O Tangent Bug usa uma **Action** (`nav_msgs_custom/action/NavigateToGoal`) que permite:
- Enviar metas em tempo de execução (requisito do enunciado)
- Receber feedback contínuo (distância, estado, heurísticas)
- Cancelar a navegação
- Receber resultado final (sucesso ou "sem caminho")

### Tópicos

| Tópico | Tipo | Direção | Descrição |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Entrada | Pose do robô |
| `/scan` | `sensor_msgs/LaserScan` | Entrada | Leituras do laser (Best Effort QoS) |
| `/cmd_vel` | `geometry_msgs/Twist` | Saída | Comandos de velocidade |
| `/navigate_to_goal` | Action | Entrada | Meta do usuário |
| `/tangent_bug_markers` | `visualization_msgs/Marker` | Saída | Meta no RViz |

## Como Executar

### Iniciar a simulação e o servidor

```bash
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 launch nav_tangent_bug tangent_bug.launch.py
```

Isso sobe: Gazebo com obstáculos, RViz, bridge, e o servidor Tangent Bug.

### Enviar metas

```bash
# Terminal 2
source ~/ros2_ws/install/setup.bash

# Cenário A: meta livre (sem obstáculo no caminho)
ros2 run nav_tangent_bug tangent_bug_client -- --x -3.0 --y 0.0

# Cenário B: meta atrás do cilindro (reta cruza cyl_1 em (3,2))
ros2 run nav_tangent_bug tangent_bug_client -- --x 5.0 --y 3.0

# Cenário C: meta distante com vários obstáculos
ros2 run nav_tangent_bug tangent_bug_client -- --x -5.0 --y -5.0

# Cenário D: meta dentro do obstáculo em U (testa no_path)
ros2 run nav_tangent_bug tangent_bug_client -- --x 6.75 --y 5.5
```

### Monitorar

```bash
# Ver feedback em tempo real
ros2 topic echo /rosout --field msg | grep -i -E "motion|boundary|goal|stuck"

# Verificar consistência de odometria
gz model -m diff_robot --pose
ros2 topic echo /odom --once --field pose.pose.position
```

## Cenários de Teste

### Cenário A: Caminho livre

O robô parte de (0,0) e vai até (-3, 0). Não há obstáculos no caminho direto.

**Comportamento esperado:** permanece em `motion_to_goal` durante toda a navegação, com d_goal diminuindo monotonicamente até `goal_reached`.

### Cenário B: Obstáculo no caminho

A reta de (0,0) a (5,3) passa pelo cilindro em (3,2) com raio 0.4 m.

**Comportamento esperado:**
1. `motion_to_goal`: vai direto até detectar o cilindro (~2 m de distância)
2. `boundary_following`: contorna o cilindro pelo lado mais curto
3. `motion_to_goal`: caminho livre detectado, vai direto à meta
4. `goal_reached`

### Cenário C: Múltiplos obstáculos

Caminho longo com vários obstáculos potencialmente no caminho.

**Comportamento esperado:** múltiplas transições MTG→BF→MTG conforme encontra e contorna obstáculos.

### Cenário D: Sem caminho (obstáculo em U)

Meta dentro do obstáculo em U fechado.

**Comportamento esperado:** o robô contorna o U, detecta volta completa, e reporta `no_path_found`.

## Parâmetros

| Parâmetro | Default | Descrição |
|---|---|---|
| `v_max` | 0.22 | Velocidade linear máxima (m/s) |
| `omega_max` | 2.84 | Velocidade angular máxima (rad/s) |
| `goal_tolerance` | 0.10 | Tolerância de chegada (m) |
| `safe_distance` | 0.50 | Distância para considerar bloqueado (m) |
| `wall_follow_distance` | 0.45 | Distância da parede no BF (m) |
| `bf_stagnation_timeout` | 5.0 | Timeout de estagnação no BF (s) |
| `physical_stuck_timeout` | 3.0 | Timeout de travamento físico (s) |
| `loop_closure_dist` | 0.3 | Distância para fechar volta completa (m) |
| `loop_closure_min_travel` | 1.5 | Mínimo percorrido antes de checar loop (m) |

## Resultados

O algoritmo demonstra os três comportamentos esperados:
- Navegação direta quando o caminho está livre
- Contorno de obstáculos quando o caminho está bloqueado
- Detecção de "sem caminho" em tempo finito quando a meta é inacessível
