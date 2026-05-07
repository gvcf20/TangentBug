# Relatório — Trabalho de Planejamento de Movimento de Robôs

## 1. Introdução

Este trabalho implementa quatro estratégias de navegação reativa para robôs com acionamento diferencial, simulados em Gazebo Harmonic com ROS 2 Jazzy. O robô utilizado é o TurtleBot3 Burger, equipado com sensor laser LDS-01 (360 amostras, alcance 0.12–3.5 m).

Os quatro exercícios abordam, progressivamente, os conceitos fundamentais de robótica móvel: seguimento de trajetória, campos potenciais, navegação com contorno de obstáculos, e coordenação multi-robô.

## 2. Plataforma

### Hardware simulado
- TurtleBot3 Burger
- v_max = 0.22 m/s, ω_max = 2.84 rad/s
- LDS-01 LIDAR: 360°, 5 Hz, alcance 3.5 m

### Software
- Ubuntu 24.04 (WSL2)
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- Python 3.12

### Decisão de arquitetura: odometria

O URDF utiliza dois plugins Gazebo separados: `DiffDrive` (apenas controle motor) e `OdometryPublisher` (única fonte de odometria e TF). Essa separação elimina divergência entre posição reportada e real, que foi identificada como causa de falhas nos testes iniciais.

## 3. Exercício 2 — Curva Paramétrica

Ver documentação detalhada em [ex2_curva_parametrica.md](ex2_curva_parametrica.md).

### Resumo

Campo vetorial com componentes normal (convergência) e tangente (circulação) aplicado à lemniscata de Bernoulli. Transição suave via tanh. Controlador a 20 Hz.

### Resultado

O robô converge e circula a lemniscata indefinidamente a partir de qualquer posição inicial.

### Comando

```bash
ros2 launch nav_parametric_curve curve_follower.launch.py
```

## 4. Exercício 3 — Campo Potencial

Ver documentação detalhada em [ex3_potencial.md](ex3_potencial.md).

### Resumo

Potencial atrativo (parabólico + cônico) + repulsivo (via laser, com transformação de frame). Meta recebida via tópico `/goal_pose`, compatível com RViz.

### Resultados

| Cenário | Resultado |
|---|---|
| Meta livre | ✔ Sucesso |
| Meta atrás de cilindro | ✔ Sucesso |
| Meta com múltiplos obstáculos | ✔ Sucesso |
| Meta dentro do U | ✘ Mínimo local |

O mínimo local em obstáculos côncavos é uma limitação fundamental do método, conforme discutido em Choset et al. (2005, Cap. 4).

### Comando

```bash
ros2 launch nav_potential_field potential_field.launch.py
# Enviar meta:
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 5.0, y: 3.0}}}"
```

## 5. Exercício 1 — Tangent Bug

Ver documentação detalhada em [ex1_tangent_bug.md](ex1_tangent_bug.md).

### Resumo

Algoritmo Tangent Bug com máquina de estados (Motion-to-Goal / Boundary-Following), heurísticas d_reach e d_followed, detecção de volta completa (no_path), e interface via ROS 2 Action.

### Resultados

| Cenário | Resultado |
|---|---|
| Meta livre | ✔ Sucesso (motion_to_goal direto) |
| Meta atrás de cilindro | ✔ Sucesso (MTG → BF → MTG) |
| Meta com múltiplos obstáculos | ✔ Sucesso |
| Meta inacessível | ✔ Detecta no_path em tempo finito |

Diferente do campo potencial, o Tangent Bug é capaz de contornar obstáculos côncavos e detectar quando não existe caminho.

### Comando

```bash
ros2 launch nav_tangent_bug tangent_bug.launch.py
# Enviar meta:
ros2 run nav_tangent_bug tangent_bug_client -- --x 5.0 --y 3.0
```

## 6. Exercício 4 — Multi-Robô

Ver documentação detalhada em [ex4_multi_robo.md](ex4_multi_robo.md).

### Resumo

Composição F = α·F_curva + β·F_rep_obstáculos + γ·F_rep_robôs. N instâncias do nó controlador, cada uma com namespaces separados. Cada robô subscreve a odometria dos outros para repulsão mútua.

### Resultado

Os robôs convergem para a lemniscata e circulam evitando obstáculos estáticos e colisões mútuas.

### Comando

```bash
ros2 launch nav_multi_robot multi_robot.launch.py n_robots:=2
```

## 7. Organização do Código

### Princípio de separação

- `nav_bringup`: infraestrutura (robô, simulação, mundos) — zero algoritmos
- `nav_common`: utilitários matemáticos reutilizáveis — zero dependência ROS (testável com pytest)
- `nav_msgs_custom`: interfaces de comunicação
- Pacotes de exercícios: um por exercício, importam de nav_common e entre si

### Reuso entre exercícios

```
nav_common
    ├── geometry.py        → usado por TODOS
    ├── diff_drive.py      → usado por exercícios 1, 2, 3, 4
    └── laser_utils.py     → usado por exercícios 1, 3, 4

nav_parametric_curve
    ├── curves.py          → usado por exercícios 2 e 4
    └── vector_field.py    → usado por exercícios 2 e 4

nav_potential_field
    └── repulsive.py       → usado por exercícios 3 e 4
```

O exercício 4 não reimplementa nada — importa diretamente dos exercícios 2 e 3.

### Testes

27 testes unitários em `nav_common` cobrindo funções de geometria e processamento de laser. Executáveis sem simulador via pytest.

## 8. Dificuldades Encontradas

### Odometria inconsistente
O maior desafio técnico foi a divergência entre posição real (Gazebo) e odometria (ROS). Resolvido isolando a publicação de odometria em um único plugin (`OdometryPublisher`).

### Wall-following em obstáculos pequenos
Cilindros de raio 0.3-0.5 m são difíceis de wall-follow porque o robô perde contato rapidamente. Resolvido com detecção de estagnação e recovery (recuar + girar).

### Plugins Gazebo Harmonic
A transição do Gazebo Classic para o Harmonic mudou nomes de plugins e convenções de tópicos. Documentação oficial incompleta. Resolvido por tentativa e erro.

### Bridge e QoS
O Gazebo publica laser com QoS "Best Effort" enquanto o padrão ROS 2 é "Reliable". Mismatch silencioso — o subscriber simplesmente não recebe dados sem mensagem de erro.

## 9. Conclusão

Os quatro exercícios foram implementados com sucesso, demonstrando progressivamente os conceitos de navegação reativa: campo vetorial (exercício 2), campos potenciais (exercício 3), algoritmos Bug com garantia de completude (exercício 1), e coordenação multi-robô (exercício 4).

A arquitetura modular com separação clara entre infraestrutura, utilitários e algoritmos permitiu reuso extensivo de código e facilitou o desenvolvimento incremental. A fatoração em pacotes ROS 2 independentes demonstra boas práticas de engenharia de software aplicadas à robótica.

## 10. Referências

1. Choset, H. et al. *Principles of Robot Motion: Theory, Algorithms, and Implementations*. MIT Press, 2005.
2. ROS 2 Jazzy Documentation. https://docs.ros.org/en/jazzy/
3. Gazebo Harmonic Documentation. https://gazebosim.org/docs/harmonic
4. TurtleBot3 e-Manual. https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
