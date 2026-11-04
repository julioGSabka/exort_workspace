# TODO — pontos da revisão de 04/11/2026

Itens para analisar **um a um**. Marque `[x]` quando resolvido e anote a decisão tomada.
Prioridade: **A** = afeta a metodologia/resultados · **B** = estrutura/qualidade · **C** = detalhe.

## Metodologia e modelo
- [ ] **A — Medir τi (torque de interação) na simulação.** Verificar o que o estado `effort` do
  `gz_ros2_control` 0.7 retorna (provável: torque comandado). Opções: sensor force/torque do Gazebo em
  junta fixa rotor→link + bridge; junta "medidora"; cálculo pela dinâmica.
- [ ] **A — Definir τi no hardware real** (AK10-9 não tem sensor de torque; estimativa τ ≈ Kt·i). Conversar com o orientador.
- [ ] **A — Bloco de atraso Ds** no controlador: buffer de [θ, θ̇, τi] com N_D = Ds/Ts e interpolação.
- [ ] **A — Malha interna de torque** (cascata; avaliar *chainable controllers* do ros2_control).
- [ ] **A — Referência de movimento θd(t), θ̇d(t)** padronizada (degrau, senoide, chirp...).
- [ ] **A — Gravidade/orientação:** o spawn com pitch −1,57 deixa o eixo da junta vertical (evidência:
  com torque zero a junta ficou parada em ~1e-12 rad por 30 s). Decidir se é o desejado.
- [ ] **A — Excitação para observar a resposta:** sem gravidade e sem perturbação a junta fica parada na
  referência. Definir como excitar (condição inicial ≠ p_ref, torque externo pelo Gazebo, referência θd(t)).
- [ ] **A — Modelo do atuador:** inércia do rotor refletida pela redução 9:1 (×81), atrito da caixa,
  dinâmica do laço de corrente; quantização do quadro MIT/CAN e atraso de comunicação.
- [ ] **B — Link do URDF é o membro robótico** (não humano): renomear macro `human_arm_weight`/`peso`
  e revisar massa/dimensões do link.
- [ ] **B — Faixa de Kv e Bv:** definir protocolo/critério para "responde adequadamente" (1º objetivo atual).
  Referência com o modelo atual: J ≈ 0,035 kg·m²; Kv = 5 → ωn ≈ 12 rad/s, ζ ≈ 0,18 (Bv = 0,1).

## Código e estrutura
- [ ] **B — Mensagem própria com header** (`exort_msgs/ImpedanceState`: θ, θ̇, τi, τd, erros, Kv, Bv, Ds,
  saturação). Hoje o plugin publica `control_msgs/JointControllerState` (não tem campo de saturação).
- [ ] **B — Perturbação externa pelo Gazebo** (ApplyJointForce/wrench), se for necessária, em vez de somar no comando.
- [ ] **B — Dependência circular** `exort_description` → `$(find exort_simulation)` no `motorAK.gazebo.xacro`:
  passar o YAML dos controladores como argumento do xacro.
- [ ] **B — `package.xml`:** remover deps não usadas (`nav_msgs`, `position_controllers`,
  `velocity_controllers`, `joint_trajectory_controller`) e declarar as usadas (`xacro`, `controller_manager`,
  `joint_state_broadcaster`...); preencher descrições TODO.
- [ ] **B — Linters do `ament_lint` falhando** (flake8 nos launch files dos dois pacotes, pep257).
- [ ] **B — Pacote `exort_experiments`:** execução automática da grade (Kv, Bv, Ds), reset do Gazebo,
  gravação por ensaio, pós-processamento (RMSE, Kef/Bef, εK/εB, C_est, Ds,lim). Instalar `pandas`.
- [ ] **B — Launch encadeado por eventos** (spawn → JSB → controlador → ensaio) em vez de `TimerAction`
  com tempos fixos (hoje o spawner do controlador sobe aos 10 s, antes do robô, que sobe aos 15 s).
- [ ] **C — `SetEnvironmentVariable` sobrescreve** `IGN_GAZEBO_RESOURCE_PATH` → usar `AppendEnvironmentVariable`.
- [ ] **C — URDF:** colisão do estator é cilindro em Z (motor está em X); literal `-0.0102` em vez de
  `rotor_altura`; `Gazebo/Grey` é material do Gazebo Classic; convenção de nome `.urdf.xacro`;
  `xmlns:gz` sem uso; `assets/a80_6gz.*` sem uso.
- [ ] **C — Identificadores em inglês**, comentários em português (padronizar).

## Repositórios e reprodutibilidade
- [ ] **B — `.repos` com `version: main`:** fixar commit/tag por campanha de ensaios.
- [ ] **C — Monorepo vs. multi-repo** (vcstool): avaliar se 4 repos compensam para uma IC.

## Feito
- [x] Controlador migrado para plugin C++ do ros2_control (`exort_control/MitImpedanceController`),
  Ts exato = período do controller_manager; nós Python antigos removidos (QoS/filas e timer em sim-time
  deixaram de existir).
- [x] Argumentos `gui` e `rviz` nos launches (necessários para rodar sem janelas).
