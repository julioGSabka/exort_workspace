# Contexto do Projeto — Exort Workspace (IC FAPESP, UNESP Sorocaba)

> Arquivo de contexto para retomar o trabalho sem reanalisar o código.
> Última atualização: 2026-11-04. Atualize ao mudar arquitetura, tópicos ou objetivos.
> **Fonte de verdade do objetivo:** o projeto FAPESP (reconsideração, versão #2 de 04/11/2026),
> resumido na §1–2. README do workspace foi removido (citava RTOS); READMEs dos pacotes atualizados em 04/11/2026.

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
- `ros2_control` + `gz_ros2_control` (hardware `GazeboSimSystem`), `controller_manager` a 1000 Hz
- Nós em **Python (rclpy)**; PlotJuggler para telemetria; `vcstool` para multi‑repo; build com `colcon`

## 4. Estrutura do código
```
~/ic_fapesp/                     repo "exort_workspace" (github julioGSabka) — .repos + este arquivo (README removido)
├── exort_project.repos          vcs import src < exort_project.repos
└── src/                         (ignorado pelo git do workspace; cada pacote é um repo próprio)
    ├── exort_description/       repo julioGSabka/exort_description
    │   ├── urdf/motorAK.xacro.urdf    world→estator(fixo)→rotor (motor_joint) + carga
    │   ├── urdf/motorAK.gazebo.xacro  bloco <ros2_control> + plugin gz_ros2_control
    │   ├── urdf/peso.xacro.urdf       macro human_arm_weight (barra 2 kg, 0,3 m) presa ao rotor
    │   ├── assets/ak10_9v2_0.stl      malha do motor (a80_6gz.* não é usada)
    │   ├── launch/exort_description.launch.py  robot_state_publisher + rviz2
    │   └── rviz/view_model.rviz
    └── exort_simulation/        repo julioGSabka/exort_simulation
        ├── launch/start_simulation.launch.py   Gazebo + spawn + bridge + joint_state_broadcaster
        ├── launch/sim_and_control.launch.py    launch principal (inclui o anterior + controle)
        ├── config/controller_config.yaml       JSB + effort_controller (JointGroupEffortController)
        ├── config/plotJug_layout.xml           layout PlotJuggler (pos/vel/effort da junta)
        ├── worlds/empty.sdf                    passo de física 1 ms, RTF 1
        └── src/ak10_controller.py | mit_command_publisher.py
```
As pastas `include/` estão vazias (ainda não há C++).

## 5. Modelo físico (exort_description)
- `motor_joint`: `continuous`, eixo X, limite de 48 Nm e 33,5 rad/s, damping 0,05, friction 0,27.
- Massas (datasheet AK10-9, 0,96 kg): estator 0,768 kg (cilindro oco, raios 49/41 mm), rotor 0,192 kg.
- Carga: `arm_link` de 2 kg, caixa de 0,02×0,3×0,03 m, fixa ao rotor com offset de 0,1 m. Gera torque
  gravitacional na junta.
- Spawn em z = 0,1 com pitch −1,57: o eixo fica horizontal e o braço pendula sob a gravidade.
- ros2_control: **command interface = effort** (emula o modo MIT); estados: position, velocity, effort.

## 6. Código de controle atual (exort_simulation)
### `ak10_controller.py` (v2, ainda sem commit)
Emula o **modo MIT** do driver AK: `τ = Kp(p_des−p) + Kd(v_des−v) + T_ff`, a 1 kHz nominal.
- `use_impedance_law=true`: Kp = Kd = 0 e **T_ff = Kv·ΔP + Bv·ΔV**, com ΔP = p_rest − p e ΔV = −v.
  Equivale à eq. (1) com θd = p_rest constante e θ̇d = 0. Kv e Bv ≥ 0, ajustáveis em runtime
  (`ros2 param set /ak10_controller kv 20.0`).
- `use_impedance_law=false`: quadro MIT cru vindo de `~/mit_command` [pos_des, vel_des, kp, kd, t_ff].
- `hold_initial_position`: torque 0 até `capture_delay` (tempo sim.); depois captura `p_rest`.
  Serviço `~/capture_rest` (Trigger).
- Segurança: torque 0 se `/joint_states` tiver mais de 0,1 s; corte de sobrevelocidade; clamp de ±48 Nm.
- Entrada `~/external_torque` (Float64), somada **depois** do clamp — sobra do teste de perturbação
  (gerador removido); inofensiva (timeout → 0), pode ser reaproveitada ou removida.
- `~/debug` = [ΔP, ΔV, T_ff, motor_effort, tau_ext, total_effort, saturated, rest_position].
- Publica em `/effort_controller/commands`.

### `mit_command_publisher.py`
Teste do MIT cru: alterna `pos_min`/`pos_max` a cada `intervalo` s
(parâmetros `intervalo, kp, kd, vel_alvo, tff, pos_min, pos_max`).

### Sequência de launch (`sim_and_control.launch.py`)
1. Seta `IGN_GAZEBO_RESOURCE_PATH`, abre o Gazebo (`-r empty.sdf`), o robot_state_publisher e o RViz.
2. t ≈ 15 s (relógio de parede): spawn de `motor_AK10`; ao terminar, sobem a bridge (`/clock`, `/tf`) e o JSB.
3. t ≈ 10 s (relógio de parede): spawner do `effort_controller` (espera até 30 s) e PlotJuggler.
4. Imediato: `ak10_controller`. Args: `kv`, `bv`, `capture_delay`, `controller`, `plotjuggler_layout`, `world_name`, `urdf_file`, `use_sim_time`.

## 7. Lacunas entre o código e a metodologia (o que falta desenvolver)
1. **Referência de movimento θd(t), θ̇d(t):** hoje a referência é só p_rest constante. É preciso
   uma trajetória padronizada e parametrizável (degrau, senoide, chirp...) dentro do controlador
   ou num nó gerador.
2. **Malha interna de torque:** hoje τd vai direto para a interface effort, sem realimentação de τi.
   Falta definir como emular a malha de torque do driver e **como medir τi no Gazebo**. Atenção:
   o estado `effort` do `GazeboSimSystem` tende a refletir o torque comandado, não o de interação.
   Opções: sensor force/torque na junta, junta extra entre rotor e carga, ou cálculo pela dinâmica.
3. **Bloco de atraso Ds:** buffer circular sobre [θ, θ̇, τi] com N_D = Ds/Ts e interpolação.
   Deve ser parâmetro do controlador (Ds = 0 é a referência).
4. **Ts constante:** no bag antigo (já apagado) o laço rodou a ~314 Hz efetivos, contra 1 kHz nominais (timer Python
   com sim-time). Com isso o atraso em amostras fica impreciso. Avaliar: atualizar o controle no callback
   do `/joint_states`, usar um controlador `ros2_control` em C++ (controller plugin), ou medir Ts real
   e indexar o buffer por tempo.
5. **Registro sincronizado** de θ, θ̇, τi, τd (e eθ, eθ̇, eτ, Ds, Kv, Bv) a cada passo do controle, num
   tópico único ou em rosbag/CSV por ensaio.
6. **Automação dos ensaios:** script que varre a grade E = (Kv, Bv, Ds), reinicia em condições
   iniciais padronizadas (reset do Gazebo), descarta a estabilização e salva cada ensaio.
7. **Pós-processamento** (Python, numpy/pandas): RMSE, erro de torque, ajuste de Kef/Bef, εK e εB,
   classificador de estabilidade C_est, Ds,lim e o mapa (Kv, Bv, Ds).
8. Preencher as descrições TODO dos `package.xml`.
9. (Feito em 04/11/2026: removidos `torque_disturbance.py`, rosbag antigo e args de perturbação; READMEs atualizados.)

## 8. Comandos úteis
```bash
cd ~/ic_fapesp && colcon build --symlink-install && source install/setup.bash
ros2 launch exort_simulation sim_and_control.launch.py kv:=20.0 bv:=0.5
ros2 service call /ak10_controller/capture_rest std_srvs/srv/Trigger
```
Estado git (04/11/2026): alterações sem commit em `exort_simulation` (launch, package.xml,
ak10_controller v2, README), `exort_description` (readme) e no workspace (README removido, CLAUDE.md novo).
