# Exercício 3 — Campo Potencial

## Objetivo

Implementar uma estratégia simples de função de potencial (Potencial Atrativo + Potencial Repulsivo) para navegar o robô em simulação entre duas posições quaisquer num ambiente com obstáculos. O laser é utilizado para o cálculo das distâncias em relação aos obstáculos e para encontrar a direção do gradiente.

## Fundamentação Teórica

### Potencial Atrativo

O potencial atrativo puxa o robô em direção à meta. Dois regimes são usados para evitar saturação:

**Regime parabólico** (d ≤ d_threshold): força proporcional à distância. O robô desacelera suavemente ao se aproximar.

```
F_att = k_att · (goal - pos)
```

**Regime cônico** (d > d_threshold): força de magnitude constante. Evita que metas distantes gerem forças gigantes que anulem o repulsivo.

```
F_att = k_att · d_threshold · (goal - pos) / d
```

### Potencial Repulsivo

O potencial repulsivo empurra o robô para longe dos obstáculos detectados pelo laser. Para cada feixe i com leitura d_i < d_0 (distância de influência):

```
|F_rep_i| = k_rep · (1/d_i - 1/d_0) / d_i²
```

A direção é oposta ao feixe (do obstáculo para o robô). A força total é a soma de todas as contribuições.

**Transformação de frames:** as forças repulsivas são calculadas no frame do laser e depois rotacionadas para o frame do mundo pelo yaw do robô. Erro nessa transformação é a causa mais comum de comportamento errático.

### Campo Total

```
F = F_att + F_rep
```

O robô converte o vetor resultante em velocidades (v, ω) usando a mesma estratégia do exercício 2.

### Limitação: Mínimos Locais

O método apresenta **mínimos locais** em obstáculos côncavos (formato U): a força atrativa puxa para dentro da concavidade, onde as forças repulsivas se equilibram com a atrativa, criando um ponto de equilíbrio que não é a meta.

Esta é uma **limitação conhecida e esperada** do método de campo potencial. O exercício 1 (Tangent Bug) resolve este cenário com garantia de completude.

### Mitigação: Wall-Following Orientado à Meta

Quando o robô fica preso (posição não muda por 3+ segundos), o nó ativa um modo de wall-following temporário com três condições de saída:

1. **Encontrou caminho melhor**: distância à meta menor que onde ficou preso
2. **Volta completa**: retornou ao ponto de partida sem melhorar
3. **Timeout**: tempo máximo esgotado (30s)

O lado de contorno alterna a cada tentativa.

## Implementação

### Pacote: `nav_potential_field`

```
nav_potential_field/
├── nav_potential_field/
│   ├── attractive.py              ← compute_attractive (parabólico + cônico)
│   ├── repulsive.py               ← compute_repulsive (laser) + compute_repulsive_from_points
│   └── potential_field_node.py    ← nó ROS com wall-follow escape
├── config/potential_params.yaml
└── launch/potential_field.launch.py
```

### Tópicos

| Tópico | Tipo | Direção | Descrição |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Entrada | Pose do robô |
| `/scan` | `sensor_msgs/LaserScan` | Entrada | Leituras do laser (Best Effort QoS) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Entrada | Meta (compatível com "2D Goal Pose" do RViz) |
| `/cmd_vel` | `geometry_msgs/Twist` | Saída | Comandos de velocidade (20 Hz) |
| `/potential_markers` | `visualization_msgs/Marker` | Saída | Meta no RViz (esfera verde) |

## Como Executar

### Iniciar a simulação

```bash
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 launch nav_potential_field potential_field.launch.py
```

Sobe: Gazebo com mundo `obstacles_simple`, RViz, bridge, e o nó potential_field.

### Enviar metas

**Opção 1 — Via RViz (recomendado):**

Clique no botão **"2D Goal Pose"** na toolbar do RViz, depois clique no ponto desejado do mapa. O robô começa a navegar imediatamente.

**Opção 2 — Via terminal:**

```bash
# Terminal 2
source ~/ros2_ws/install/setup.bash

# Cenário A: meta em linha reta livre
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 4.0, y: 0.0}}}"

# Cenário B: meta atrás de cilindro (cyl_1 em (3,2))
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 5.0, y: 3.0}}}"

# Cenário C: meta em região com múltiplos obstáculos
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: -3.0, y: -4.0}}}"

# Cenário D: meta dentro do U (teste de mínimo local)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 6.75, y: 5.5}}}"
```

### Visualizar no RViz

No RViz, adicione **Marker** no tópico `/potential_markers` para ver a meta como esfera verde.

### Monitorar

```bash
# Ver logs (meta alcançada, avisos de mínimo local)
ros2 topic echo /rosout --field msg | grep -i -E "meta|mínimo|wall|stuck"

# Verificar taxa de controle
ros2 topic hz /cmd_vel
```

## Cenários de Teste

### Cenário A: Meta livre

Meta em (4, 0), sem obstáculos no caminho direto.

**Resultado esperado:** robô vai em linha reta, chega em ~18 segundos (4m / 0.22 m/s).

### Cenário B: Meta atrás de obstáculo convexo

Meta em (5, 3), reta cruza perto do cilindro em (3, 2).

**Resultado esperado:** a força repulsiva desvia o robô do cilindro. Trajetória curva mas chega à meta.

### Cenário C: Múltiplos obstáculos

Meta em (-3, -4), vários obstáculos potencialmente no caminho.

**Resultado esperado:** desvio de múltiplos obstáculos com trajetória suave.

### Cenário D: Mínimo local (obstáculo em U)

Meta em (6.75, 5.5), dentro do obstáculo em U.

**Resultado esperado:** o robô possivelmente fica preso no mínimo local. O wall-follow tenta escapar, mas pode não conseguir em obstáculos côncavos profundos. Este é um **resultado válido** que demonstra a limitação do método.

## Parâmetros

| Parâmetro | Default | Descrição |
|---|---|---|
| `k_att` | 1.0 | Ganho atrativo |
| `k_rep` | 1.0 | Ganho repulsivo |
| `d0` | 1.2 | Distância de influência dos obstáculos (m) |
| `d_threshold` | 1.5 | Transição parabólico/cônico do atrativo (m) |
| `goal_tolerance` | 0.15 | Distância para considerar "chegou" (m) |
| `v_max` | 0.22 | Velocidade linear máxima (m/s) |
| `omega_max` | 2.84 | Velocidade angular máxima (rad/s) |
| `wall_follow_distance` | 0.35 | Distância da parede no wall-follow (m) |
| `wall_follow_max_duration` | 30.0 | Timeout do wall-follow (s) |

## Resultados

| Cenário | Resultado |
|---|---|
| Meta em linha reta livre | ✔ Sucesso |
| Meta atrás de cilindro | ✔ Sucesso (contorna) |
| Meta atrás de caixa | ✔ Sucesso (contorna) |
| Meta com múltiplos obstáculos | ✔ Sucesso |
| Meta dentro do U | ✘ Mínimo local (limitação esperada) |

### Comparação com Tangent Bug

| Aspecto | Campo Potencial | Tangent Bug |
|---|---|---|
| Completude | Não garantida | Garantida |
| Mínimo local em U | Fica preso | Contorna ou detecta no_path |
| Suavidade da trajetória | Suave | Pode ter transições bruscas |
| Complexidade | Simples | Mais complexo |
| Tempo de computação | Baixo | Moderado |
