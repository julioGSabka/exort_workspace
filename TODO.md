# TODO — pontos da revisão de 04/11/2026

Itens para analisar **um a um**. Marque `[x]` quando resolvido e anote a decisão tomada.
Prioridade: **A** = afeta a metodologia/resultados · **B** = estrutura/qualidade · **C** = detalhe.

## Metodologia e modelo
- [ ] **A — Tratar a inércia e o atrito na medição de Kef/Bef** (eq. 8). Ensaio de 08/11/2026 (Kv = 5, Bv = 0,1,
  A = 1, f = 1): ajuste direto recuperou J = 0,0354, b = 0,052, Fc = 0,267; Re(Z) = 3,23 (esperado 3,61) e
  Im(Z)/ω = 0,357 (0,150 sem atrito). Com A = 2, Re(Z) = 3,51 (esperado 3,61): a diferença cai com a amplitude, o que reforça a hipótese do atrito.
- [ ] **B — Perda de amostras no rosbag:** 0,8% sem janelas (172 de 21.776), 2 a 3% com janelas abertas: checar `--qos-profile-overrides`/cache.
- [ ] **B — Massa e dimensões do `robot_link`:** revisar os valores (hoje 2 kg, 0,02×0,3×0,03 m, centro a 0,1 m do eixo).
- [ ] **A — Medir τi (torque de interação) na simulação.** Verificar o que o estado `effort` do
  `gz_ros2_control` 0.7 retorna (provável: torque comandado). Opções: sensor force/torque do Gazebo em
  junta fixa rotor→link + bridge; junta "medidora"; cálculo pela dinâmica.
- [ ] **A — Definir τi no hardware real** (AK10-9 não tem sensor de torque; estimativa τ ≈ Kt·i). Conversar com o orientador.
- [ ] **A — Bloco de atraso Ds** no controlador: buffer de [θ, θ̇, τi] com N_D = Ds/Ts e interpolação.
- [ ] **A — Malha interna de torque** (cascata; avaliar *chainable controllers* do ros2_control).
- [ ] **A — Referência de movimento θd(t), θ̇d(t)** padronizada (degrau, senoide, chirp...).
- [ ] **A — Gravidade/orientação:** o spawn com pitch −1,57 deixa o eixo da junta vertical (evidência:
  com torque zero a junta ficou parada em ~1e-12 rad por 30 s). Decidir se é o desejado.
- [ ] **A — Modelo do atuador:** inércia do rotor refletida pela redução 9:1 (×81), atrito da caixa,
  dinâmica do laço de corrente; quantização do quadro MIT/CAN e atraso de comunicação.
- [ ] **B — Faixa de Kv e Bv:** definir protocolo/critério para "responde adequadamente" (1º objetivo atual).
  Referência com o modelo atual: J ≈ 0,035 kg·m²; Kv = 5 → ωn ≈ 12 rad/s, ζ ≈ 0,18 (Bv = 0,1).

## Código e estrutura
- [ ] **C — PlotJuggler pede dois cliques ao abrir** (Yes e OK): não há opção de linha de comando para pular.
- [ ] **B — Perturbação externa pelo Gazebo** (ApplyJointForce/wrench), se for necessária, em vez de somar no comando.
- [ ] **B — Dependência circular** `exort_description` → `$(find exort_simulation)` no `motorAK.gazebo.xacro`:
  passar o YAML dos controladores como argumento do xacro.
- [ ] **B — Linters do `ament_lint` falhando** (flake8 nos launch files dos dois pacotes, pep257).
- [ ] **B — `exort_experiments`:** falta varrer a grade (Kv, Bv, Ds) e calcular RMSE, εK/εB, C_est, Ds,lim.
  O ensaio individual e o gráfico já existem.
- [ ] **B — Launch encadeado por eventos** (spawn → JSB → controlador → ensaio) em vez de `TimerAction`
  com tempos fixos (hoje o spawner do controlador sobe aos 10 s, antes do robô, que sobe aos 15 s).
- [ ] **C — `SetEnvironmentVariable` sobrescreve** `IGN_GAZEBO_RESOURCE_PATH` → usar `AppendEnvironmentVariable`.
- [ ] **C — URDF:** colisão do estator é cilindro em Z (motor está em X); literal `-0.0102` em vez de
  `rotor_altura`; `Gazebo/Grey` é material do Gazebo Classic; convenção de nome `.urdf.xacro`;
  `xmlns:gz` sem uso; `assets/a80_6gz.*` sem uso.
- [ ] **C — Identificadores em inglês**, comentários em português (padronizar).

## Repositórios e reprodutibilidade
- [ ] **B — `.repos` com `version: main`:** fixar commit/tag por campanha de ensaios.
- [ ] **C — Monorepo vs. multi-repo** (vcstool): avaliar se 6 repos compensam para uma IC.

## Feito
- [x] Controlador migrado para plugin C++ do ros2_control (`exort_control/MitImpedanceController`),
  Ts exato = período do controller_manager; nós Python antigos removidos (QoS/filas e timer em sim-time
  deixaram de existir).
- [x] Argumentos `gui` e `rviz` nos launches (necessários para rodar sem janelas).
- [x] Mensagem própria `exort_msgs/ImpedanceState` com header, publicada em `~/state` (08/11/2026).
  Campos τi e Ds entram quando existirem medição de τi e bloco de atraso.
- [x] Link renomeado para `robot_link` (macro e arquivo `robot_link.xacro.urdf`) (08/11/2026).
- [x] `package.xml` dos pacotes: dependências reais e descrições (08/11/2026). A dependência de
  `exort_description` em `exort_simulation` continua não declarada porque é a dependência circular listada acima.
- [x] Excitação por torque senoidal imposto e script de ensaio com gráfico (08/11/2026).
