# Exercício 4 — Multi-Robô

## Objetivo

Fazer uma composição dos controladores dos exercícios 2 e 3 de forma a ter 2 ou mais robôs navegando para convergir e circular a mesma curva paramétrica. O ambiente deve ter obstáculos estáticos fora da curva alvo e as funções de potencial devem ser utilizadas para evitar estes obstáculos. Os robôs também devem utilizar as funções de potencial para evitar colisões entre eles. É permitido que cada robô saiba a posição dos demais no ambiente.

## Fundamentação Teórica

### Composição de Campos Vetoriais

O campo total aplicado a cada robô é a **soma ponderada** de três componentes:

```
F_total = α · F_curva + β · F_rep_obstáculos + γ · F_rep_robôs
```

Onde:

**F_curva** (do exercício 2): campo vetorial que atrai e faz o robô circular a lemniscata. Combina componente normal (convergência) e tangente (circulação) com pesos variáveis pela distância.

**F_rep_obstáculos** (do exercício 3): potencial repulsivo calculado a partir do LaserScan. Cada feixe com leitura abaixo de d_0 contribui com uma força empurrando o robô para longe do obstáculo.

**F_rep_robôs** (novo): potencial repulsivo calculado a partir das posições conhecidas dos outros robôs. Usa a mesma fórmula do potencial repulsivo, mas com as posições dos outros robôs como "obstáculos pontuais":

```
Para cada robô j ≠ i, com distância d_ij:
    se d_ij < d_0_robot:
        |F_rep| = k_rep_robot · (1/d_ij - 1/d_0_robot) / d_ij²
        Direção: do robô j para o robô i (afasta)
```

### Pesos da Composição

- **α** (curva): peso base 1.0. Sempre ativo.
- **β** (obstáculos): peso base 1.0. Garante desvio de paredes e objetos estáticos.
- **γ** (robôs): peso 1.5 (maior que os outros). Prioriza evitar colisão entre robôs, que é mais perigosa que perder tracking da curva temporariamente.

### Quebra de Simetria

Se dois robôs se encontram frontalmente sobre a curva, as forças repulsivas mútuas são simétricas e podem causar deadlock. A simetria é quebrada por:

- Posições iniciais diferentes (espalhados equidistantemente num raio de 3 m)
- Fases diferentes ao longo da curva (cada robô converge para um ponto diferente)

### Comunicação entre Robôs

O enunciado permite que cada robô saiba a posição dos demais. Implementação via ROS 2: cada robô publica sua odometria em `/<namespace>/odom` e subscreve a dos outros. Não há coordenação centralizada — cada robô toma decisões locais com informação global de posições.

## Implementação

### Pacotes modificados

**`nav_bringup`** (novo mundo e launch multi-robô):
```
nav_bringup/
├── worlds/obstacles_multi.world         ← obstáculos fora da lemniscata
└── launch/sim_multi_robot.launch.py     ← spawna N robôs com namespaces
```

**`nav_multi_robot`** (novo pacote):
```
nav_multi_robot/
├── nav_multi_robot/
│   ├── composition.py         ← compute_composed_field
│   └── multi_robot_node.py    ← nó por robô
├── config/multi_robot_params.yaml
└── launch/multi_robot.launch.py
```

### Reuso de Código

O `nav_multi_robot` **não reimplementa nada** — importa diretamente:
- `compute_field` de `nav_parametric_curve.vector_field`
- `compute_repulsive` de `nav_potential_field.repulsive`
- `compute_repulsive_from_points` de `nav_potential_field.repulsive`
- `force_to_twist` de `nav_common.diff_drive`

### Arquitetura Multi-Robô no Gazebo

Cada robô é spawnado com tópicos prefixados via URDF modificado em runtime:

```
robot_0:  /robot_0/cmd_vel, /robot_0/odom, /robot_0/scan, /robot_0/tf
robot_1:  /robot_1/cmd_vel, /robot_1/odom, /robot_1/scan, /robot_1/tf
robot_2:  /robot_2/cmd_vel, /robot_2/odom, /robot_2/scan, /robot_2/tf
```

O launch `sim_multi_robot.launch.py` usa `OpaqueFunction` para gerar dinamicamente N spawn + bridge + robot_state_publisher.

### Tópicos por Robô

| Tópico | Tipo | Direção | Descrição |
|---|---|---|---|
| `/robot_N/odom` | Odometry | Entrada (próprio) + Saída (para outros) | Pose |
| `/robot_N/scan` | LaserScan | Entrada | Laser (Best Effort QoS) |
| `/robot_N/cmd_vel` | Twist | Saída | Comandos de velocidade |
| `/robot_N/tf` | TFMessage | Entrada | Transformações |

## Como Executar

### Iniciar com 2 robôs (padrão)

```bash
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 launch nav_multi_robot multi_robot.launch.py
```

### Iniciar com 3 robôs

```bash
ros2 launch nav_multi_robot multi_robot.launch.py n_robots:=3
```

### Verificar que todos os robôs estão ativos

```bash
# Terminal 2
source ~/ros2_ws/install/setup.bash

# Listar nós
ros2 node list | grep multi_robot

# Verificar tópicos de cada robô
ros2 topic list | grep robot_

# Odom de cada robô
ros2 topic echo /robot_0/odom --once --field pose.pose.position
ros2 topic echo /robot_1/odom --once --field pose.pose.position

# Cmd_vel sendo publicado
ros2 topic hz /robot_0/cmd_vel
ros2 topic hz /robot_1/cmd_vel
```

### Monitorar no RViz

No RViz:
1. Mude o **Fixed Frame** para `odom` (ou `robot_0/odom` se necessário)
2. Adicione displays **LaserScan** para `/robot_0/scan` e `/robot_1/scan`
3. Adicione displays **Odometry** para `/robot_0/odom` e `/robot_1/odom`
4. Adicione **Marker** em `/curve_marker` (se publicado)

### Parar

```bash
# Ctrl+C no terminal do launch, depois:
pkill -9 -f "gz sim"
```

## Mundo `obstacles_multi.world`

Obstáculos posicionados **fora** da lemniscata (a=2.0, curva vai de -2 a +2 em x):

| Obstáculo | Posição | Tamanho |
|---|---|---|
| cyl_1 | (4, 3) | raio 0.4 |
| cyl_2 | (-4, 2) | raio 0.5 |
| box_1 | (3, -3) | 1.2×0.6, rotação 0.3 rad |
| box_2 | (-3, -3) | 0.8×0.8 |

Os obstáculos estão a pelo menos 3 m do centro, garantindo que não bloqueiem a curva alvo.

## Parâmetros

| Parâmetro | Default | Descrição |
|---|---|---|
| `n_robots` | 2 | Número de robôs |
| `curve_type` | `lemniscate` | Tipo de curva |
| `curve_scale` | 2.0 | Escala da curva (parâmetro a) |
| `alpha` | 1.0 | Peso do campo da curva |
| `beta` | 1.0 | Peso da repulsão de obstáculos |
| `gamma` | 1.5 | Peso da repulsão entre robôs |
| `k_normal` | 1.5 | Ganho de convergência da curva |
| `k_tangent` | 1.0 | Ganho de circulação da curva |
| `convergence_radius` | 3.0 | Raio de transição convergência/circulação |
| `k_rep_obs` | 1.0 | Ganho repulsivo de obstáculos |
| `d0_obs` | 1.2 | Distância de influência dos obstáculos (m) |
| `k_rep_robot` | 2.0 | Ganho repulsivo inter-robô |
| `d0_robot` | 1.5 | Distância de influência inter-robô (m) |
| `v_max` | 0.22 | Velocidade linear máxima (m/s) |
| `omega_max` | 2.84 | Velocidade angular máxima (rad/s) |
| `control_rate` | 20.0 | Frequência do loop de controle (Hz) |

## Resultados Esperados

### Comportamento com 2 robôs

1. Ambos partem de posições opostas (raio 3 m)
2. Convergem para a lemniscata
3. Quando se aproximam (d < d0_robot), a repulsão mútua os afasta
4. Estabelecem circulação com espaçamento natural

### Comportamento com obstáculos

Os robôs desviam de obstáculos estáticos via potencial repulsivo do laser, sem sair significativamente da curva. Quando passam perto de um obstáculo, a trajetória é momentaneamente perturbada mas retorna à curva.

### Possíveis problemas

- **Deadlock simétrico**: dois robôs se encontram frontalmente. Mitigado pelas posições iniciais diferentes e pela assimetria natural da lemniscata.
- **Oscilação perto de obstáculo + outro robô**: quando γ·F_rob e β·F_obs competem. Ajustar pesos resolve.
- **Colisão em alta velocidade**: se os robôs se aproximam muito rápido. O `d0_robot=1.5 m` dá margem suficiente com v_max=0.22 m/s.
