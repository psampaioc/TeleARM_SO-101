# TeleARM — guia do projeto

Este é o documento de referência do checkout: estado atual, comandos, pipeline, segurança, mapa de ficheiros, hardware e fontes. O [README da raiz](../README.md) fica como entrada rápida. Esta atualização de controlo ainda precisa de validação física.

## Índice

1. [Arranque e segurança](#arranque-e-segurança)
2. [Estado e evidência](#estado-e-evidência)
3. [Pipeline completo](#pipeline-completo)
4. [O que a mão mede e como vira comando](#o-que-a-mão-mede-e-como-vira-comando)
5. [LeRobot, cinemática e motores](#lerobot-cinemática-e-motores)
6. [Mapa do repositório](#mapa-do-repositório)
7. [Hardware, calibração e materiais](#hardware-calibração-e-materiais)
8. [Fontes e proveniência](#fontes-e-proveniência)
9. [Decisões e limites do âmbito](#decisões-e-limites-do-âmbito)
10. [Material local de estudo](#material-local-de-estudo)

## Arranque e segurança

O entry point atual inicia **teleoperação live**: não existe pré-visualização sem braço. Ao iniciar, o programa abre a webcam, conecta o follower, lê os motores e pede manutenção da posição enquanto espera pela mão. Executa apenas com o operador ao lado, sem carga, braço apoiado, trajeto livre e corte físico de energia acessível. A primeira mão válida estabelece uma referência; amostras seguintes podem comandar o braço. Na perda, o programa pede manutenção da posição lida: isso não comprova paragem física.

Na raiz do projeto, inicia um container temporário sem reconstruir a imagem:

```bash
docker compose run --rm --name so101-hand lerobot bash
```

Dentro do container:

```bash
cd /workspace/Brosnantest
python -B hand_camera_teleop.py
```

`Esc` na janela pede encerramento normal; não é um corte de emergência. Em movimento inesperado ou falha de leitura/escrita, usa o corte físico. O encerramento por erro não garante que a ligação/torque foi desligado. Não executar calibração, setup ou jog enquanto a teleoperação estiver ligada, nem abrir outro processo para a câmara ou porta serial.

O serviço Compose `lerobot` usa o container temporário `so101-hand` e a imagem `desk-arm-lerobot:0.7.0-single-process`. O projeto e `data/` estão montados com escrita em `/workspace`, portanto as alterações nesses caminhos ficam no host. Pacotes instalados fora do projeto ficam na camada do container e são perdidos ao sair; para os preservar, alterar o Dockerfile e fazer build deliberadamente. Python do projeto, LeRobot, câmara e motores executam-se no container. Ao sair do primeiro shell com `exit`, o container termina e é removido. Enquanto estiver aberto, outro terminal pode entrar com `docker exec -it so101-hand bash`.

O serviço executa como `root`, preservando o acesso atual aos dispositivos; ficheiros criados nesse shell podem pertencer a `root`. Para editar ficheiros como o utilizador do host, com o container aberto, entrar noutro terminal com `docker exec -it --user "$(id -u):$(id -g)" so101-hand bash`.

`XAUTHORITY` indica o ficheiro de autenticação da sessão gráfica que permite abrir a janela da câmara. A pasta de runtime do utilizador já está montada, pelo que não se monta separadamente esse ficheiro temporário. Iniciar `deskarm` na sessão gráfica atual usa os valores atuais de `DISPLAY` e `XAUTHORITY`.

Antes de qualquer arranque físico, seguir estes passos:

1. Confirmar que o operador está junto ao braço e consegue alcançar o corte físico.
2. Retirar cargas, apoiar o braço, libertar o percurso e deixar cabos com folga.
3. Confirmar que `/dev/ttyACM0` é o follower e que nenhum outro processo usa a câmara ou a serial.
4. Confirmar que existe a calibração LeRobot indicada abaixo; não a substituir nem recalibrar durante live.
5. Iniciar uma única instância e observar a janela. Interromper fisicamente se houver movimento inesperado, imagem parada ou falha de comunicação.

## Estado e evidência

O sistema é um protótipo experimental. Em 12/11/2026 o operador reportou que o braço acompanhou a mão numa sessão. Na mesma sessão ocorreu timeout na leitura do Motor 1 e o encerramento terminou com erro; não se confirmou o estado final do torque. O movimento foi observado uma vez, mas operação repetível, segurança, cinemática XYZ, abertura do gripper e recuperação de falhas continuam sem aceitação. Este relato é histórico; não é verificação do estado físico atual.

| Afirmação | Estado conhecido |
|---|---|
| Webcam, MediaPipe e landmarks | Implementados; o operador já viu o esqueleto. A resolução/FPS que a câmara realmente aplica não são confirmados pelo programa. |
| LeRobot lê o follower, resolve a ação e envia posições | Implementado no código. Uma chamada aceite não prova que todos os servos executaram corretamente. |
| Braço seguiu a mão | Um movimento visível foi reportado pelo operador; ainda não é validação repetível. |
| XYZ em centímetros, com escala real | Não implementado nem validado. X/Y são relativos à imagem; “Z” é variação de tamanho aparente. |
| Gripper segue a mão | A versão atual usa mudança de ângulo 2D e consome cada amostra uma vez; ainda não foi validada no braço. |
| Direção da mão, M1/M4/M5 e pausa | Implementação experimental verificada em simulação; direção, sensibilidade e pausa física ainda não foram validadas no braço. |
| Orientação 3D da palma, feedback visual ou Rerun | Não implementados nesta cadeia. |
| Leitura serial e encerramento após falha | Não confiáveis ainda; timeout no Motor 1 e torque final desconhecido. |
| Latência p50/p95 live | Não é medida de forma fiável pelo programa atual. Não reutilizar as métricas de versões anteriores. |

## Pipeline completo

```text
Container existente
  → hand_camera_teleop.py
  → webcam /dev/video0 → OpenCV → MediaPipe (21 landmarks)
  → deltas 2D e ângulos heurísticos
  → camera_processor.py (schema LeRobot; valores inalterados)
  → referência cartesiana → limites → gripper → IK PlaCo
  → SO101Follower / LeRobot → serial Feetech /dev/ttyACM0 → servos
```

Um processo Python tem três threads:

1. **Captura:** OpenCV abre `/dev/video0`, pede V4L2, 640×480 e 30 FPS, e coloca frames BGR numa fila de até cinco imagens. `cap.set()` não confirma que o modo foi aceite. Se a fila encher, o código descarta o frame novo.
2. **Tracking e janela:** converte BGR para RGB, chama MediaPipe Tasks em modo VIDEO (uma mão; limiares 0,5), calcula características, desenha esqueleto com números e seta B e envia amostras para uma fila de até 100 itens. `detect_for_video()` é síncrono nesta thread.
3. **Braço:** lê posições do follower, usa a amostra válida mais recente, aplica os processadores LeRobot e escreve alvos. Frequência nominal do ciclo: 45 Hz. A escrita de `Goal_Position` não prova que a posição física foi atingida.

O comando `deskarm` abre um shell; o container não inicia teleoperação sozinho. O entry point inicializa hardware ao ser executado e pode fazê-lo também durante importação. A webcam é solicitada a 30 FPS e o ciclo do braço tenta 45 Hz; são frequências diferentes, não uma medição de latência ponta a ponta.

## O que a mão mede e como vira comando

MediaPipe fornece 21 landmarks de imagem normalizados (`x`,`y` entre 0 e 1, tipicamente). O helper `to_vec()` descarta `z`. Índices: `0` pulso; `1–4` polegar; `5–8` indicador; `9–12` médio; `13–16` anelar; `17–20` mindinho. O programa desenha todos, mas usa só alguns nos cálculos.

### Posição e escala aparente

- A posição segue o landmark **0** (pulso), não o centro da palma. A primeira deteção válida e cada retorno após pausa criam uma nova origem visual e registam as posições dos motores; não há gesto PALM.
- As mudanças de `x₀` e `y₀` são deslocamentos em coordenadas normalizadas de imagem, não centímetros.
- A estimativa chamada `depth_estimate()` soma os comprimentos 2D das cadeias `0–1–2–3–4` e `0–5–6–7–8`. A diferença em relação à amostra inicial é usada como proxy de escala aparente.
- Rodar ou abrir a mão também altera os comprimentos projetados; a escala não mede distância física à lente. O `z` estimado pelo MediaPipe é usado na orientação, não como profundidade medida.
- Posição, escala, ângulo da garra e ângulos de orientação passam por média exponencial: 90% do valor anterior + 10% da amostra atual. A média reduz tremor e acrescenta atraso.

O avanço combina o deslocamento horizontal do pulso projetado segundo a direção da mão com a variação de escala aparente. Aproximar a mão da câmara aumenta a escala e pede avanço mesmo com a mão de lado. O alvo é colocado no plano radial do braço, que acompanha M1; o deslocamento vertical do pulso altera a altura. O fator 0,5 produz valores interpretados pela pipeline como metros, mas não torna a medição da webcam métrica. A acumulação da escala aparente pode causar deriva. A caixa cartesiana configurada em ±1 m e o passo máximo de 0,10 m não são um envelope mecânico comprovado; um salto rejeitado coloca o controlo em espera.

### Gripper e rotação

- O gripper usa o ângulo 2D entre os vetores pulso→ponta do polegar (`0→4`) e pulso→ponta do indicador (`0→8`). Envia a diferença do ângulo anterior, multiplicada por 0,5 no ciclo do robô.
- `GripperVelocityToJoint` aplica `speed_factor=20`, soma a variação à leitura atual e limita o alvo numérico a 0–100. Isto não é uma medida direta da abertura da mão nem prova qual extremo físico abre/fecha.
- A direção 3D estimada usa o vetor do polegar `2→4`. A seta na imagem parte do ponto 0, paralela à projeção do polegar, e fica curta quando este aponta para a câmara. Os ângulos horizontal e vertical comandam M1 e M4; o ponto 5 fornece uma referência para estimar o roll de M5. Os pontos podem ser estimados incorretamente quando estão ocultos, e a referência de roll varia com a direção da mão. Sinais e precisão ainda não foram validados fisicamente.
- A IK usa a origem de `wrist_link`, no eixo de M4, e não impõe orientação (`orientation_weight=0`). M2/M3 tentam posicionar M4. Depois da IK, o alvo de M4 compensa a variação de M2/M3 para acompanhar a inclinação pedida, sujeito aos limites da calibração e do URDF. A ponta da garra pode deslocar-se ao rodar M4; a compensação foi verificada no modelo, mas ainda precisa de teste físico.
- Uma geometria ou vetores inválidos são tratados como ausência da mão. A perda ou imagem antiga provoca pausa e um pedido de manutenção; a primeira amostra válida após o regresso reancora, e uma amostra posterior volta a mover.
- Um comando já em transmissão não pode ser cancelado; erros de leitura/escrita ou o pedido de manutenção não garantem paragem física.
- `WAIT HAND LIMIT` indica alvo M1/M4/M5 fora da calibração; `WAIT MOTOR LIMIT` indica leitura fora dela. `WAIT MOVE LIMIT` indica salto cartesiano rejeitado. `WAIT IK` indica uma solução com erro acima de 5 mm ou um QP inviável reportado pelo PlaCo; neste último caso o terminal mostra as leituras e os alvos. `HAND - WAIT` indica que uma mão foi detetada mas a amostra não é válida para controlo. O programa espera por uma amostra aceitável; erros de comunicação continuam a terminar o controlo. Em `WAIT MOTOR LIMIT` o último objetivo pode continuar ativo. Nos outros casos, a manutenção é apenas um pedido, não uma confirmação de paragem.

Na retomada, a mão e as juntas são ancoradas no mesmo ciclo; a referência cartesiana é capturada pela cadeia na amostra seguinte. Os landmarks world são usados para estimar direção, não profundidade medida por sensor. Não há PALM/FIST/PINCH nem handedness usado.

### Funções para estudar no código

| Símbolo | Ficheiro | Função |
|---|---|---|
| `frame_capture()` | `Brosnantest/hand_camera_teleop.py` | Lê frames da webcam e gere a fila de imagem. |
| `to_vec()` | mesmo | Extrai `x,y` normalizados de um landmark; ignora `z`. |
| `angle_between_vectors()` | mesmo | Calcula ângulo 2D pelo produto escalar e `arccos`. |
| `direction_angles()` | mesmo | Estima a direção do polegar e os ângulos para M1/M4/M5. |
| `depth_estimate()` | mesmo | Soma os segmentos 2D do polegar e indicador como proxy de tamanho aparente. |
| `frame_display()` | mesmo | Corre MediaPipe, suaviza características, desenha pontos e publica amostras. |
| `robot_process()` | mesmo | Lê o follower, remapeia os campos, executa processadores/IK e envia ações. |
| `MapCameraHandActionToRobotAction.action()` | `Brosnantest/camera_processor.py` | Identidade: não transforma os valores numéricos. |
| `transform_features()` | mesmo | Declara os campos do schema que LeRobot espera. |

O nome de registo `map_phone_action_to_robot_action` e a remoção de campos antigos `phone.*` são compatibilidade herdada; não indicam entrada por telefone. O remapeamento de eixos está em `robot_process()`, não no adapter.

## LeRobot, cinemática e motores

A cadeia de processadores atual, por ordem:

1. **`MapCameraHandActionToRobotAction`:** adaptador de schema local, ação identidade.
2. **`EEReferenceAndDelta`:** calcula a pose de `wrist_link` pela leitura articular e fixa uma referência após cada reancoragem. Multiplica alvos pelos ganhos e soma à referência; não espera PALM.
3. **`EEBoundsAndSafety`:** limita posição à caixa configurada e rejeita passos acima de 0,10 m. O controlo aguarda uma amostra aceitável quando a proteção rejeita um salto; não é um limitador gradual.
4. **`GripperVelocityToJoint`:** integra a velocidade angular do gripper num alvo numérico 0–100.
5. **`InverseKinematicsEEToJoints`:** LeRobot e PlaCo resolvem M2/M3 para a posição desejada de M4, mantendo M1/M4/M5 nos alvos calculados da mão. A orientação da garra não é imposta à IK. Antes do envio, rejeita-se uma solução com erro de posição superior a 5 mm. O gripper é independente da IK.

`SO101Follower` do LeRobot lê/escreve as posições `.pos` por Feetech. O LeRobot é dono do protocolo serial, calibração de motor e conversão. O ciclo é alvo de posição mais readback, não um PID em Python. Ao conectar, o LeRobot configura PID no servo (P=16, I=0, D=32 na versão 0.7 analisada).

| Item | Papel e limite |
|---|---|
| Calibração LeRobot `teleArm_follower.json` | Interpretação/conversão calibrada do follower em `use_degrees=True`; não calibra webcam/mão/XYZ. A implementação `DEGREES` v0.7 analisada não faz clamp explícito a `range_min/range_max`. |
| URDF `so101_new_calib.urdf` | Geometria, juntas, frames e limites do solver; o alvo atual é `wrist_link`. Não mede o estado físico nem certifica batentes. |
| Conversão servo e limites do solver | Camadas diferentes. O código não monta explicitamente uma interseção de limites entre calibração JSON e URDF. |
| Readback/escrita serial | A leitura informa estado articular para o ciclo seguinte; uma escrita aceite não comprova execução física correta. |

Mapeamento do hardware: Motor 1 `shoulder_pan`/base; Motor 2 `shoulder_lift`; Motor 3 `elbow_flex`; Motor 4 `wrist_flex`; Motor 5 `wrist_roll`; Motor 6 gripper. M1/M4/M5 recebem alvos diretos e são considerados pela IK de M2/M3; M6 atua separadamente. Os limites locais do URDF para Motores 2 e 3 foram alargados num ensaio temporário; não são limites mecânicos confirmados nem devem ser interpretados como envelope seguro.

## Mapa do repositório

Para aprender o runtime, abre os ficheiros nesta ordem:

1. `Brosnantest/hand_camera_teleop.py` — entry point e maior parte da lógica.
2. `Brosnantest/camera_processor.py` — schema adapter LeRobot.
3. `teleop/models/so101/so101_new_calib.urdf` — modelo de cinemática; precisa dos meshes relativos em `assets/`.
4. `compose.yaml`, `Dockerfile`, `teleop/requirements.txt` — ambiente e dependências.
5. `scripts/setup_follower_motors.py` e `scripts/calibrate_follower.py` — operações manuais separadas, não executadas pelo entry point.

```text
.
├── AGENTS.md                         regras locais privadas para agentes
├── README.md                         quickstart curto
├── Brosnantest/
│   ├── hand_camera_teleop.py         entry point live e tracking
│   └── camera_processor.py           schema adapter LeRobot
├── teleop/
│   ├── requirements.txt              dependências diretas do runtime
│   └── models/so101/
│       ├── so101_new_calib.urdf      URDF carregado pelo solver
│       ├── LICENSE                   licença upstream mantida com o modelo
│       └── assets/*.stl              meshes usados pelo URDF
├── scripts/
│   ├── setup_follower_motors.py      setup manual
│   └── calibrate_follower.py         calibração manual
├── assets/printing/so101-follower/   quatro pacotes locais de impressão; fora do runtime
├── data/                             modelo MediaPipe e calibração privados/ignorados
├── records/                          recibos/fotos/documentos privados/ignorados
├── docs/README.md                    este guia canónico
├── docs/guides/                      estudo local, ignorado pelo Git
├── docs/transcripts/                 transcrições locais, ignoradas pelo Git
├── compose.yaml                      configuração do container
├── Dockerfile                        imagem de runtime
└── teleop/requirements.txt           dependências diretas
```

`data/models/hand_landmarker.task` e `data/lerobot-home/calibration/robots/so_follower/teleArm_follower.json` são inputs locais requeridos. As capturas e medições antigas autorizadas foram removidas. `data/` também contém HOME/cache do container; não apagar como limpeza documental. `records/` é privado e não foi aberto. O contexto de build ignora dados, records, docs e meshes, enquanto o Compose monta o projeto e `data/` com escrita.

O container usa Ubuntu 24.04, Python 3.12, `lerobot[feetech,kinematics]==0.7.0`, MediaPipe 0.10.35 e PyTorch CPU. `teleop/requirements.txt` não é lockfile completo; OpenCV, NumPy e PlaCo chegam por dependências. `gpus: all` no Compose não altera o backend CPU de inferência.

## Hardware, calibração e materiais

### SO-101 deste projeto

| Peça | Escolha atual | Estado |
|---|---|---|
| Braço | SO-101 follower, sem leader | Montado para este projeto. |
| Servos | Seis STS3215-C018, 12 V, 30 kg·cm, relação 1/345 | Compra reportada e recibo local; confirmar etiquetas antes de energizar. |
| Fonte | Regulada 12 V, mínimo 5 A | Utilizador reportou possuir uma fonte 12 V/5 A. Etiqueta, regulação, conector, polaridade e compatibilidade do controlador continuam por confirmar. |
| Controlador | Placa Feetech/Waveshare usada pelo LeRobot | Não substituir por ESP32-only sem validar eletricamente. |
| Webcam | `/dev/video0`, solicitação 640×480/30 FPS | Foco/zoom automáticos foram reportados; modo real não confirmado pelo script. |

Não tratar torque de bloqueio como payload nominal. Verificar tensão, polaridade, corrente e controlador com a documentação elétrica antes de energizar. Não alterar cabos sob alimentação. O URDF/calibração não são e-stop nem garantia dos limites mecânicos.

### Materiais de impressão locais

`assets/printing/so101-follower/` contém atualmente três ficheiros preservados, nenhum usado pelo runtime:

- `ender-model.zip` e `individual-parts.zip` — pacotes locais de impressão.
- `so-101_impressao_CR/Ender_Follower_SO101.stl` — ficheiro local recuperado do clone de referência.

Os 13 STL em `teleop/models/so101/assets/` são meshes do URDF ativo: 11 correspondem às peças impressas e dois representam os servos STS3215. Os seus hashes correspondem aos assets do SO-101 no commit oficial `eecbe3e0a9ebb23e25ad7b2759b03884c6660903`, também indicado como fonte do pacote de peças individuais. Os STL do pacote de impressão são exports separados e não são byte a byte iguais aos meshes do URDF; essa diferença de ficheiro não prova que a geometria das peças seja incompatível.

Comparei a cadeia cinemática local com o URDF oficial desse commit: origens, eixos, juntas, frames e limites coincidem, exceto pelos limites locais temporariamente alargados de `shoulder_lift` (±1,94 rad) e `elbow_flex` (±1,71 rad). Portanto, trocar apenas os nomes/caminhos dos meshes não corrigiria uma discrepância de cinemática; trocar o URDF inteiro também reverteria esses dois limites.

O STL solto `so-101_impressao_CR/Ender_Follower_SO101.stl` é byte a byte igual ao membro com o mesmo nome em `ender-model.zip`; foi mantido como entrada direta para o slicer, sem extração do ZIP.

## Fontes e proveniência

### Componentes ativos e referências

| Origem | Commit/versão | Uso |
|---|---|---|
| [BrosnanYuen/hand_pose_lerobot_teleop](https://github.com/BrosnanYuen/hand_pose_lerobot_teleop/tree/425a4fba62b6c7c07fc3968a18d00da99bc193f2) | `425a4fba62b6c7c07fc3968a18d00da99bc193f2` | Base dos dois ficheiros `Brosnantest/`, adaptados ao LeRobot e MediaPipe do container; cabeçalhos Apache-2.0 preservados. |
| [Hugging Face LeRobot](https://github.com/huggingface/lerobot/tree/v0.7.0) | `v0.7.0` | Follower, Feetech, calibração/conversão, processadores e cinemática; dependência instalada. |
| [Google MediaPipe](https://github.com/google-ai-edge/mediapipe) | dependência instalada | Modelo/runtime de landmarks; o ficheiro `.task` é dado local em `data/models/`. |
| [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/7629d2ad9853d10fb903093a33ef6114099d97e5) | `7629d2ad9853d10fb903093a33ef6114099d97e5`, 2026-08-05 | Origem do URDF/meshes SO-101; Apache-2.0. O checkout local foi removido em 2026-11-12. |

Referências de pesquisa anteriores, sem imports nem dependências ativas: [AFK-Kireeti/Lerobot-SO101](https://github.com/AFK-Kireeti/Lerobot-SO101/tree/32c561abd05354fb51aae2bdf0e74df8b0df9ff2) (`32c561abd05354fb51aae2bdf0e74df8b0df9ff2`), [ABMI-software/hand_controlLerobot](https://github.com/ABMI-software/hand_controlLerobot/tree/52c7f0d926d0f66107ae91b94ceb900766f3009d) (`52c7f0d926d0f66107ae91b94ceb900766f3009d`), [sruthihsr/SO101-Gesture-Teleoperation](https://github.com/sruthihsr/SO101-Gesture-Teleoperation/tree/ac5468b603a65c556634893ef8564873206734c4) (`ac5468b603a65c556634893ef8564873206734c4`), [ros2/ros2_documentation](https://github.com/ros2/ros2_documentation), [NVIDIA/Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T) e o fork local [psampaioc/SO-ARM100](https://github.com/psampaioc/SO-ARM100). O clone `upstream/SO-ARM100/` ocupava cerca de 470 MB e foi removido com autorização; os ficheiros locais de impressão foram preservados em `assets/printing/so101-follower/`.

O commit TheRobotStudio observado era `7629d2ad9853d10fb903093a33ef6114099d97e5`, data `2026-08-05T12:21:21+03:00`, assunto “Add Robonine as worldwide supplier to Kits list (#164)”. O checkout removido tinha `origin=https://github.com/psampaioc/SO-ARM100.git` e `upstream=https://github.com/TheRobotStudio/SO-ARM100.git`. A documentação upstream de montagem/calibração SO-101 está em [LeRobot](https://huggingface.co/docs/lerobot/so101).

### URDF e meshes

O ficheiro ativo vem de `Simulation/SO101/so101_new_calib.urdf`. Os caminhos relativos `assets/` foram mantidos. A licença original não foi modificada e permanece em `teleop/models/so101/LICENSE`.

| Ficheiro relativo a `teleop/models/so101/` | Bytes | SHA-256 da cópia de origem |
|---|---:|---|
| `so101_new_calib.urdf` | 16231 | `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c` |
| `LICENSE` | 11357 | `c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4` |
| `assets/base_motor_holder_so101_v1.stl` | 1877084 | `8cd2f241037ea377af1191fffe0dd9d9006beea6dcc48543660ed41647072424` |
| `assets/base_so101_v2.stl` | 471584 | `bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d` |
| `assets/motor_holder_so101_base_v1.stl` | 1129384 | `31242ae6fb59d8b15c66617b88ad8e9bded62d57c35d11c0c43a70d2f4caa95b` |
| `assets/motor_holder_so101_wrist_v1.stl` | 1052184 | `887f92e6013cb64ea3a1ab8675e92da1e0beacfd5e001f972523540545e08011` |
| `assets/moving_jaw_so101_v1.stl` | 1413584 | `785a9dded2f474bc1d869e0d3dae398a3dcd9c0c345640040472210d2861fa9d` |
| `assets/rotation_pitch_so101_v1.stl` | 883684 | `9be900cc2a2bf718102841ef82ef8d2873842427648092c8ed2ca1e2ef4ffa34` |
| `assets/sts3215_03a_no_horn_v1.stl` | 865884 | `75ef3781b752e4065891aea855e34dc161a38a549549cd0970cedd07eae6f887` |
| `assets/sts3215_03a_v1.stl` | 954084 | `a37c871fb502483ab96c256baf457d36f2e97afc9205313d9c5ab275ef941cd0` |
| `assets/under_arm_so101_v1.stl` | 1975884 | `d01d1f2de365651dcad9d6669e94ff87ff7652b5bb2d10752a66a456a86dbc71` |
| `assets/upper_arm_so101_v1.stl` | 1303484 | `475056e03a17e71919b82fd88ab9a0b898ab50164f2a7943652a6b2941bb2d4f` |
| `assets/waveshare_mounting_plate_so101_v2.stl` | 62784 | `e197e24005a07d01bbc06a8c42311664eaeda415bf859f68fa247884d0f1a6e9` |
| `assets/wrist_roll_follower_so101_v1.stl` | 1439884 | `4b17b410a12d64ec39554abc3e8054d8a97384b2dc4a8d95a5ecb2a93670f5f4` |
| `assets/wrist_roll_pitch_so101_v2.stl` | 2699784 | `6c7ec5525b4d8b9e397a30ab4bb0037156a5d5f38a4adf2c7d943d6c56eda5ae` |

O URDF local tem SHA-256 `dbc2c5a814310716702dac93e9808a735e34286b8d1ae248c9caf3d2360dac9a`. Em 10/11/2026 os limites dos Motores 2 (`shoulder_lift`, ±1,94 rad) e 3 (`elbow_flex`, ±1,71 rad) foram alargados para um ensaio; o JSON de calibração não foi alterado. São limites temporários do solver, não limites mecânicos confirmados nem um envelope de comandos seguro.

### BOM e limites da compra

A BOM upstream que serviu de referência é uma lista histórica do follower/leader com preços e servos 7,4 V; **não é a configuração comprada** e pode estar desatualizada. Para este projeto, a decisão é seis STS3215-C018 de 12 V e fonte regulada de pelo menos 12 V/5 A; a fonte disponível foi reportada pelo operador, mas compatibilidade física ainda requer verificação. Leader excluído. Recibo em `records/hardware/servo-purchase-receipt.pdf` é privado e ignorado pelo Git; não incluir o recibo em documentação pública.

## Decisões e limites do âmbito

- O projeto usa só o follower SO-101 já montado; não existe braço leader.
- O runtime de mão é o entry point de dois ficheiros em `Brosnantest/`, com MediaPipe, LeRobot e o URDF SO-101. A decisão é provisória até movimento e serial ficarem repetíveis.
- Manter LeRobot como proprietário da comunicação Feetech, leitura/escrita e conversão calibrada. Não implementar driver ou conversão paralelos.
- Não chamar a escala aparente “profundidade real” nem apresentar XYZ como métrico.
- Não declarar XYZ/gripper aceite até haver ensaios live repetíveis e resolução da falha serial/encerramento.
- Inclinação 3D, pose normalizada da palma, depth camera, feedback Rerun e treino ACT/VLA estão fora do runtime atual.
- ROS 2, MuJoCo, receiver TCP, líder, Rerun e câmara ESP32 não participam na cadeia ativa. O antigo código ROS e o seu workspace foram removidos com autorização em 12/11/2026.

## Material local de estudo

`docs/guides/` e `docs/transcripts/` contêm um guia local sobre o curso SO-101 da NVIDIA e as suas transcrições. São materiais de estudo, ignorados pelo Git e não necessários ao programa. Foram preservados; o ficheiro de guia tem um lock do LibreOffice, por isso não foi movido nem editado.

O mapa acima descreve o que está em uso e o que é material local. Os dados, registos, modelo MediaPipe, calibração, meshes/licença e pacotes de impressão foram mantidos; esta reorganização trata apenas da documentação e dos seus links.
