# Contexto do Projeto — Exort Workspace (IC FAPESP, UNESP Sorocaba)

> Arquivo de contexto para retomar o trabalho sem reanalisar o código.
> Última atualização: 2026-11-08. Atualize ao mudar arquitetura, tópicos ou objetivos.
> **Fonte de verdade do objetivo:** o projeto FAPESP (reconsideração, versão #2 de 04/11/2026),
> resumido na §1–2. README do workspace foi removido (citava RTOS); READMEs dos pacotes atualizados em 04/11/2026.
> Pendências da revisão: **TODO.md**.

## 1. O projeto (versão atual, submetida à FAPESP)
**Título:** *Efeitos de Atrasos Temporais na Estabilidade do Controle de Impedância: uma abordagem
baseada em simulação física com Gazebo e ROS 2.*
- Discente: Julio Gustavo Brendler Sabka · Orientador: Prof. Dr. Felix Mauricio Escalante Ortega
- UNESP – ICT Sorocaba, Depto. Eng. de Controle e Automação; recursos do laboratório GASI.
- Versões do projeto: #1 em 02/04/2026; #2 (reconsideração) em 04/11/2026.

**Mudança de escopo:** a camada RTOS **foi abandonada**. Continua o uso do atuador
**CubeMars AK10-9 KV100 em modo MIT**, primeiro em simulação (Gazebo + ROS 2).

**Objetivo principal:** investigar quantitativamente como **atrasos temporais na realimentação**
afetam a **estabilidade** e a **transmissibilidade da impedância virtual** (quanto da rigidez e do
amortecimento definidos no controlador aparece de fato na resposta) num sistema rotacional de
1 GDL, para várias combinações de rigidez Kv e amortecimento Bv.

**Objetivos específicos:**
1. Modelar e implementar no Gazebo + ROS 2 o sistema de 1 GDL com controle de impedância,
   numa configuração experimental controlada e reprodutível.
2. Caracterizar a resposta para vários (Kv, Bv) **sem atraso** (referência, Ds = 0).
3. Avaliar sistematicamente o efeito de vários atrasos Ds × (Kv, Bv) sobre estabilidade e transmissibilidade.
4. Identificar as condições de atraso e de parâmetros que levam à perda de estabilidade ou à
   degradação da impedância, ou seja, os limites de operação estável.

**Produto final esperado:** um **mapa experimental (Kv, Bv, Ds) → estabilidade/transmissibilidade**,
com o limite de atraso `Ds,lim` para cada par (Kv, Bv), mostrando se a impedância se degrada
antes, junto ou depois da perda de estabilidade. Os resultados irão para relatórios e para artigos
em conferências de robótica e mecatrônica.

## 2. Metodologia definida no projeto (o que o código precisa implementar)
### 2.1 Arquitetura de controle (impedância indireta, malhas em cascata)
```
θd, θ̇d ─►(+)─ eθ, eθ̇ ─► [Loop externo: impedância]  τd = Kv·(θd − θs) + Bv·(θ̇d − θ̇s)
                                     │ τd
                                     ▼
                         (+)─ eτ = τd − τi,s ─► [Loop interno: torque (motor driver)] ─► τcmd ─► AK10-9 + ambiente (Gazebo)
                                                                                                  │ θ, θ̇, τi
                    y_s(t) = y(t − Ds),  y = [θ, θ̇, τi]ᵀ   ◄── [Atraso nos estados] ◄──────────────┘
```
- O **mesmo atraso Ds** é aplicado a posição, velocidade **e** torque de interação antes de chegarem
  às malhas (eq. 2/4). A estrutura do controlador não muda entre ensaios: só Ds, Kv e Bv variam.
- Atraso discreto: buffer histórico com `N_D = Ds / Ts` amostras (eq. 5). Quando Ds não for múltiplo
  de Ts, usar **interpolação temporal**. Para isso **Ts precisa ser constante e conhecido**.
- Malha interna de torque: realimenta τi (torque de interação medido na saída do atuador).

### 2.2 Protocolo experimental
- Conjunto de ensaios: `E = {(Kv(i), Bv(i), Ds(j)) | i = 1..N_KB, j = 0..N_D}` (eq. 6). Para cada (Kv, Bv),
  roda-se primeiro Ds = 0 e depois valores crescentes de Ds.
- Ficam constantes: modelo mecânico, parâmetros do atuador, Ts, estrutura do controle,
  **referência de movimento pré-definida**, condições iniciais padronizadas e ambiente de simulação.
- Pode haver um intervalo inicial de estabilização, descartado na análise.
- Registrar **sincronizado com o controlador**, no mínimo: θ(t), θ̇(t), τi(t), τd(t).
- Fazer repetições independentes quando couber (média e desvio-padrão).

### 2.3 Métricas
- RMSE de posição e de velocidade; erro de torque (τd − τi).
- **Impedância efetiva:** ajustar `τi ≈ Kef·eθ + Bef·eθ̇` (eq. 8), por exemplo por mínimos quadrados, e
  calcular `εK = |Kef − Kv|/|Kv|·100`, `εB = |Bef − Bv|/|Bv|·100` (eq. 9). Usar métricas absolutas
  ou normalizadas quando Kv ou Bv forem próximos de 0.
- **Estabilidade:** o ensaio é estável se as variáveis ficam limitadas e os erros convergem, sem
  oscilação sustentada nem crescimento persistente. Indicadores: amplitude máxima de θ, amplitude
  máxima de eθ, RMSE, evolução de eτ. Os critérios são fixados antes dos ensaios e são os mesmos
  para todos. Transitórios não contam como instabilidade.
- **Limite experimental:** `Ds,lim(i) = max{Ds(j) | C_est(Kv(i), Bv(i), Ds(j)) = 1}` (eq. 7). É um
  limite empírico, não teórico.

### 2.4 Cronograma (12 meses)
| Tarefa | Meses |
|---|---|
| T1 Revisão bibliográfica + definição do protocolo (faixas de Kv, Bv, Ds; métricas) | 1 |
| T2 Modelagem no Gazebo + controlador de impedância em ROS 2, validado sem atraso | 2–5 |
| T3 Caracterização da impedância sem atraso (referência) | 5–7 |
| T4 Ensaios com atraso, limites de estabilidade, análise estatística, relatórios e artigos | 6–12 |

Bibliografia-chave: Hogan (1985) sobre impedância; Colgate & Schenkel (1997) sobre passividade
em sistemas amostrados; Lawrence (1993) sobre estabilidade × transparência; Escalante et al.
(trabalhos do orientador) sobre impedância e transparência markovianas em exoesqueletos.

## 3. Stack
- ROS 2 **Humble**, Gazebo **Ignition/Fortress** (`ros_gz_sim`, plugins `ignition-gazebo-*`)
- `ros2_control` 2.54 + `gz_ros2_control` 0.7 (hardware `GazeboSimSystem`), `controller_manager` a 1000 Hz
- Controlador em **C++** (plugin `controller_interface`, `generate_parameter_library`, `realtime_tools`);
  testes com gtest/gmock; PlotJuggler para telemetria; `vcstool` para multi‑repo; build com `colcon`

## 4. Estrutura do código
```
~/ic_fapesp/                     repo "exort_workspace" (github julioGSabka) — .repos + CLAUDE.md + TODO.md
├── exort_project.repos          vcs import src < exort_project.repos
├── results/                     ensaios gravados (ignorado pelo git)
├── TODO.md                      pendências da revisão de 04/11/2026 (analisar uma a uma)
└── src/                         (ignorado pelo git do workspace; cada pacote é um repo próprio)
    ├── exort_description/       repo julioGSabka/exort_description
    │   ├── urdf/motorAK.xacro.urdf    world→estator(fixo)→rotor (motor_joint) + link
    │   ├── urdf/motorAK.gazebo.xacro  bloco <ros2_control> + plugin gz_ros2_control
    │   ├── urdf/robot_link.xacro.urdf macro robot_link (barra 2 kg, 0,3 m) — é o MEMBRO ROBÓTICO
    │   ├── assets/ak10_9v2_0.stl      malha do motor (a80_6gz.* não é usada)
    │   ├── launch/exort_description.launch.py  robot_state_publisher + rviz2 (arg `rviz`)
    │   └── rviz/view_model.rviz
    ├── exort_msgs/              repo julioGSabka/exort_msgs — msg/ImpedanceState.msg
    ├── exort_experiments/       ensaio sem janelas, análise e gráficos (exort_experiments/, test/)
    ├── exort_control/           repo julioGSabka/exort_control
    │   ├── include/exort_control/mit_impedance_law.hpp         lei pura (sem ROS), testável
    │   ├── include/exort_control/mit_impedance_controller.hpp  plugin ros2_control
    │   ├── include/exort_control/sine_excitation.hpp           gerador puro do torque externo
    │   ├── src/mit_impedance_controller.cpp
    │   ├── src/mit_impedance_controller_parameters.yaml        parâmetros (generate_parameter_library)
    │   ├── exort_control.xml                                   descrição pluginlib
    │   └── test/  test_mit_impedance_law | test_mit_impedance_controller | test_load_...
    └── exort_simulation/        repo julioGSabka/exort_simulation
        ├── launch/start_simulation.launch.py   Gazebo (arg `gui`) + spawn + bridge + JSB
        ├── launch/sim_and_control.launch.py    inclui o anterior + spawner do controlador + PlotJuggler
        ├── config/controller_config.yaml       JSB + mit_impedance_controller (kv, bv, limites)
        ├── config/plotJug_layout.xml           PlotJuggler: posição, velocidade e torques de ~/state
        └── worlds/empty.sdf                    passo de física 1 ms, RTF 1
```
Não há mais nós Python (ak10_controller.py e mit_command_publisher.py foram removidos em 04/11/2026).

## 5. Modelo físico (exort_description)
- `motor_joint`: `continuous`, eixo X, limite de 48 Nm e 33,5 rad/s, damping 0,05, friction 0,27.
- Massas (datasheet AK10-9, 0,96 kg): estator 0,768 kg (cilindro oco, raios 49/41 mm), rotor 0,192 kg.
- Link: `robot_link` de 2 kg, caixa de 0,02×0,3×0,03 m, centro a 0,1 m do eixo. **Nesta fase é o membro
  robótico, não o humano** (revisão de massa e dimensões na TODO.md).
  Inércia total em torno do eixo ≈ 0,0353 kg·m² (braço 0,0152 + m·d² 0,02 + rotor 0,00016).
- Spawn em z = 0,1 com pitch −1,57 (rotação em Y): o eixo X da junta fica **vertical** e o link gira num
  plano horizontal, **sem torque gravitacional**. Evidência: com torque zero por ~30 s a junta ficou em
  ~1e-12 rad (teste de 04/11/2026). Decidir se é o desejado (TODO.md).
- ros2_control: **command interface = effort** (emula o modo MIT); estados: position, velocity, effort.
- Dinâmica com o controlador: J·θ̈ + (Bv + 0,05)·θ̇ + Kv·(θ − θ0) + atrito = 0 →
  ωn = √(Kv/J), ζ = (Bv + 0,05)/(2√(Kv·J)). Ex.: Kv = 5, Bv = 0,1 → ωn ≈ 12 rad/s, ζ ≈ 0,18.

## 6. Controlador atual (exort_control/MitImpedanceController)
**Fase atual (decidida com o orientador):** MIT com **kp = kd = 0**, controlando só o torque com
`t_ff = Kv·(p_ref − p) + Bv·(0 − v)`; `p_ref` = posição no **início da simulação** (capturada no 1º
`update()` após a ativação). Objetivo imediato: **definir a faixa de Kv e Bv em que controlador e modelo
respondem adequadamente**.
- Roda no laço do `controller_manager` (Ts = 1 ms exato, confirmado: `time_step = 0.001`). Sem tópicos
  entre planta e lei de controle → Ds = 0 é atraso zero de verdade.
- **Excitação (decidida em 08/11/2026):** torque senoidal imposto `τ_ext = rampa·A·sin(φ)`, somado depois da
  saturação e invisível à lei de controle; parâmetros `excitation_{amplitude,frequency,start_delay,ramp_time}`.
  Agente acoplado fica para depois; atrito segue em aberto. Amplitude de 0 para > 0 recomeça a sequência
  (é assim que `run_excitation_test --attach` dispara o ensaio na simulação aberta).
- Comando: `motor_joint/effort`; estados: `position`, `velocity` (procurados nessa ordem).
- Parâmetros (`generate_parameter_library`, validados): `joint` (read-only), `kv` ≥ 0, `bv` ≥ 0,
  `max_effort` > 0 (48), `max_velocity` > 0 (33,5). Kv/Bv/limites mudam em execução:
  `ros2 param set /mit_impedance_controller kv 20.0` (aplicado no próximo update, sem alocação).
- Proteções: sobrevelocidade (não empurra no sentido do movimento acima de max_velocity), saturação
  ±max_effort; estado não finito → torque 0; ativação/desativação → torque 0.
- `~/state` (`exort_msgs/ImpedanceState`, 1 kHz, `header.stamp` em sim-time): time_step, position_reference,
  position, velocity, position_error, velocity_error, kv, bv, feedforward_torque, raw_torque,
  commanded_torque, external_torque, total_torque, saturated, overspeed.
- Testes (30 em exort_control, 6 em exort_experiments): lei pura, controlador com interfaces locais e mensagem publicada, carga via pluginlib com
  `mock_components/GenericSystem`.

### Sequência de launch (`sim_and_control.launch.py`)
1. `IGN_GAZEBO_RESOURCE_PATH`, Gazebo (`-r`, ou `-r -s` com `gui:=false`), robot_state_publisher, RViz opcional.
2. t ≈ 15 s (parede): spawn de `motor_AK10` → bridge (`/clock`, `/tf`) e spawner do JSB.
3. t ≈ 10 s (parede): spawner do `mit_impedance_controller` (espera até 30 s pelo CM) com YAML temporário
   via `--param-file`; quando o spawner termina, abre o PlotJuggler (opcional).
Args: `kv`, `bv`, `amplitude`, `frequency`, `start_delay`, `ramp_time`, `gui`, `rviz`, `plotjuggler`, `plotjuggler_layout`, `world_name`, `urdf_file`, `use_sim_time`.

## 7. Lacunas entre o código e a metodologia
Lista completa e priorizada em **TODO.md**. Principais: critério de "responde adequadamente"; τi medido; bloco de atraso Ds;
malha interna de torque; referência θd(t); registro por ensaio; automação da grade; pós-processamento.

## 8. Comandos úteis
```bash
cd ~/ic_fapesp && colcon build --symlink-install && source install/setup.bash
colcon test --packages-select exort_control && colcon test-result --verbose
ros2 launch exort_simulation sim_and_control.launch.py kv:=20.0 bv:=0.5
ros2 launch exort_simulation sim_and_control.launch.py gui:=false rviz:=false plotjuggler:=false
ros2 run exort_experiments run_excitation_test --kv 5 --bv 0.1 --amplitude 1 --frequency 1 --duration 20
ros2 run exort_experiments run_excitation_test --attach --amplitude 2  # usa a simulação já aberta, com janelas
ros2 run exort_experiments plot_excitation_test results/<pasta_do_ensaio>
ros2 control list_controllers
ros2 topic echo /mit_impedance_controller/state
```
