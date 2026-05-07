# Exercício 2 — Curva Paramétrica

## Objetivo

Implementar um controlador que faça um robô com acionamento diferencial convergir e circular eternamente uma curva plana definida por equações paramétricas (que não seja um círculo ou uma elipse), em simulação sem obstáculos.

## Fundamentação Teórica

### Lemniscata de Bernoulli

A curva escolhida é a **lemniscata de Bernoulli**, com formato de ∞ (infinito). Não é círculo nem elipse (requisito do enunciado), possui curvatura variável e cruzamento na origem.

Equações paramétricas:

```
x(t) = a · cos(t) / (1 + sin²(t))
y(t) = a · sin(t) · cos(t) / (1 + sin²(t))
```

O parâmetro `a` controla o tamanho — com a=2.0, a curva vai de -2 a +2 em x.

Uma curva alternativa (cardioide) também está implementada:

```
x(t) = a · (2cos(t) - cos(2t))
y(t) = a · (2sin(t) - sin(2t))
```

### Abordagem por Campo Vetorial

Em vez de perseguir um ponto específico da curva, a estratégia é construir um **campo vetorial** F(x,y) que cobre todo o plano. Em cada ponto, o campo indica a direção que o robô deve seguir.

O campo é composto por dois componentes:

**Componente Normal (convergência):** Aponta do robô para o ponto mais próximo da curva. Faz o robô se aproximar.

```
F_normal = k_normal · tanh(d / convergence_radius) · N̂
```

Onde d é a distância à curva e N̂ é o vetor unitário do robô ao ponto mais próximo. O `tanh` satura suavemente: longe → puxa forte; em cima → quase zero.

**Componente Tangente (circulação):** Aponta na direção tangente à curva no ponto mais próximo. Faz o robô circular.

```
F_tangente = k_tangent · (1 - 0.5·tanh(d / convergence_radius)) · T̂
```

Onde T̂ é o vetor tangente unitário. O peso é complementar ao normal: em cima da curva → circula forte; longe → circula fraco.

**Campo Total:**

```
F = F_tangente + F_normal
```

A transição entre convergência e circulação é automática e suave, sem chaveamentos.

### Conversão para Robô Diferencial

O campo vetorial dá uma direção desejada θ_d = atan2(Fy, Fx). O robô diferencial converte para velocidades (v, ω):

```
erro = θ_d - θ_atual
ω = k_omega · erro (saturado)
v = v_max · cos(erro) quando |erro| < π/2, senão 0
```

O `cos(erro)` faz o robô parar de andar e só girar quando está desalinhado, depois acelerar quando aponta na direção certa.

## Implementação

### Pacote: `nav_parametric_curve`

```
nav_parametric_curve/
├── nav_parametric_curve/
│   ├── curves.py              ← Lemniscate, Cardioid (definições paramétricas)
│   ├── vector_field.py        ← compute_field (normal + tangente ponderados)
│   └── curve_follower_node.py ← nó ROS: odom → campo → twist → cmd_vel
├── config/curve_params.yaml
└── launch/curve_follower.launch.py
```

### Tópicos

| Tópico | Tipo | Direção | Descrição |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Entrada | Pose do robô |
| `/cmd_vel` | `geometry_msgs/Twist` | Saída | Comandos de velocidade (20 Hz) |
| `/curve_marker` | `visualization_msgs/Marker` | Saída | Curva alvo no RViz (verde) |

### Pipeline de Controle

```
/odom (pose) → compute_field(x, y, curva) → (Fx, Fy) → force_to_twist(Fx, Fy, yaw) → /cmd_vel
```

Executado a 20 Hz por um timer. O callback de odometria só guarda o estado mais recente; o timer faz o controle — desacoplando taxa do sensor da taxa do controlador.

## Como Executar

### Iniciar a simulação

```bash
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 launch nav_parametric_curve curve_follower.launch.py
```

Sobe: Gazebo com mundo vazio, RViz, bridge, e o nó curve_follower.

### Visualizar a curva no RViz

No RViz, clique **Add → By topic → /curve_marker → Marker**. A lemniscata aparece em verde.

### Monitorar

```bash
# Terminal 2
source ~/ros2_ws/install/setup.bash

# Verificar taxa do controlador
ros2 topic hz /cmd_vel

# Ver comandos sendo publicados
ros2 topic echo /cmd_vel

# Verificar que o nó está rodando
ros2 node list | grep curve
```

### Trocar parâmetros em tempo real

```bash
ros2 param set /curve_follower v_max 0.3
ros2 param set /curve_follower k_normal 2.0
ros2 param set /curve_follower curve_scale 3.0
```

### Trocar para cardioide

Edite `config/curve_params.yaml`:
```yaml
curve_type: 'cardioid'
```
Ou na linha de comando ao lançar (requer relançar).

## Parâmetros

| Parâmetro | Default | Descrição |
|---|---|---|
| `curve_type` | `lemniscate` | Tipo de curva (`lemniscate` ou `cardioid`) |
| `curve_scale` | 2.0 | Escala da curva (parâmetro `a`) |
| `k_normal` | 1.5 | Ganho de convergência para a curva |
| `k_tangent` | 1.0 | Ganho de circulação ao longo da curva |
| `convergence_radius` | 3.0 | Raio de transição convergência/circulação |
| `v_max` | 0.22 | Velocidade linear máxima (m/s) |
| `omega_max` | 2.84 | Velocidade angular máxima (rad/s) |
| `k_omega` | 2.0 | Ganho proporcional do controlador angular |
| `control_rate` | 20.0 | Frequência do loop de controle (Hz) |
| `log_trajectory` | true | Salvar trajetória em CSV |
| `log_file` | `/tmp/curve_trajectory.csv` | Arquivo de log |

## Resultados

O robô converge suavemente para a lemniscata a partir de qualquer posição inicial e circula indefinidamente no formato ∞. A trajetória é visível no RViz como setas vermelhas de odometria sobre a curva verde.

### Análise da Trajetória

O arquivo CSV de log pode ser usado para gerar gráficos:

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('/tmp/curve_trajectory.csv')
plt.figure(figsize=(10, 6))
plt.plot(df['x'], df['y'], 'r-', alpha=0.5, label='Trajetória')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title('Convergência para a Lemniscata')
plt.savefig('docs/figuras/ex2_trajetoria.png')
plt.show()
```
