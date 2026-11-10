# SO-101 controlado pela mão: resumo técnico

Este documento explica a implementação atual do protótipo e os seus limites conhecidos. Descreve o código, não afirma que a teleoperação já passou todos os testes físicos.

## Em poucas palavras

Uma webcam observa a mão. O MediaPipe estima 21 pontos da mão em cada frame. O nosso código usa esses pontos para reconhecer palma/punho, medir um alvo de translação relativo e calcular a abertura da garra. Quando o utilizador apresenta a palma durante três frames, o sistema guarda a posição da mão e a pose atual do braço como referências. Depois transforma os deslocamentos da mão num alvo XYZ e pede ao solver de cinemática inversa do LeRobot as cinco posições articulares necessárias. O LeRobot envia as posições ao SO-101 pela placa serial Feetech, usando a calibração guardada.

## Fluxo do sinal

```text
Webcam /dev/video0
  → OpenCV lê 640×480 até 30 FPS e converte BGR para RGB
  → MediaPipe Hand Landmarker estima 21 pontos da mão
  → código do projeto calcula gesto, posição relativa, profundidade aproximada e ângulo da garra
  → palma aceita: guarda referência da mão + posição medida do braço
  → alvo XYZ relativo
  → LeRobot RobotKinematics calcula as cinco juntas do braço juntas
  → LeRobot SO101Follower envia ações .pos pela serial/Feetech
  → motores respondem e LeRobot fornece novas leituras
```

O processo roda no container do projeto: Ubuntu 24.04, Python 3.12, LeRobot 0.7.0, MediaPipe 0.10.35 e uma compilação CPU do PyTorch. O programa usa OpenCV/V4L2 para abrir `/dev/video0`. A inferência de mão usa MediaPipe CPU/XNNPACK; o facto de o Compose expor uma GPU não significa que este modelo esteja a inferir na GPU.

## Quem faz o quê

| Parte | Implementação e responsabilidade |
|---|---|
| Câmara | OpenCV abre `/dev/video0` com backend V4L2 e pede 640×480/30 FPS. A janela redimensiona para 800×600. A imagem e os pontos estão espelhados para parecer uma vista de selfie; a deteção e os comandos continuam a usar as coordenadas originais. |
| Deteção da mão | MediaPipe Hand Landmarker e o modelo `.task` local estimam 21 landmarks 2D e coordenadas 3D relativas da mão. O modelo estima geometria; as regras PALM/FIST e da garra são funções Python do projeto, não classes de gesto fornecidas pelo modelo. |
| Extração dos pontos | `teleop/hand_controller.py` valida os 21 pontos, calcula as medidas e desenha cada ponto com o número 0–20 na imagem. |
| Estado e mapeamento | `teleop/controller.py` gere espera, ativação, âncora, deslocamentos XYZ, gripper, limites e comandos de movimento. |
| Cinemática | `teleop/kinematics.py` usa a API `RobotKinematics` do LeRobot com o URDF SO-101 local para FK/IK, valida as posições e limita o passo articular. |
| Motores e calibração | `teleop/lerobot_adapter.py` usa `SO101Follower` do LeRobot para a serial, leitura e escrita `.pos`. O JSON de calibração define IDs 1–6 e faixas dos encoders. A aplicação não implementa o protocolo Feetech. |
| Modelo do braço | O URDF e as malhas em `teleop/models/so101/` vêm do TheRobotStudio/SO-ARM100 no commit indicado em `teleop/SOURCES.md`, sob Apache-2.0. Há uma alteração local temporária nos limites do modelo para os motores 2 e 3; a calibração guardada pelo LeRobot não foi alterada. |

## Como são usados os 21 pontos

Os números seguem o índice de landmarks do MediaPipe: 0 é o pulso; 1–4 são o polegar; 5–8 o indicador; 9–12 o médio; 13–16 o anelar; 17–20 o mínimo. Todos aparecem no overlay. Para cada cálculo são usados apenas alguns:

| Medida | Pontos e regra | Efeito |
|---|---|---|
| Posição na imagem | O ponto 0 (pulso) fornece `x,y` normalizados entre 0 e 1. | Deslocar o pulso na imagem gera deslocamento relativo horizontal/vertical. Não se calcula o centroide dos 21 pontos: abrir os dedos deslocaria esse centro mesmo com a palma parada. |
| Aproximação/afastamento | Distância 2D entre o pulso 0 e a base do dedo médio 9, dividida pelo valor guardado quando a palma é aceite. | A mudança relativa do tamanho aproxima a profundidade. É apenas um proxy visual da webcam, não uma medição em metros. Rotação, perspetiva e deformação da mão também alteram essa distância. |
| Palma ou punho | Nas coordenadas 3D relativas do MediaPipe: `dist(2,12) / dist(5,17)`. Até 0,7 classifica FIST; a partir de 1,1 classifica PALM; entre os limiares fica HAND. | PALM precisa de três frames consecutivos para ativar ou reancorar. FIST é reconhecido imediatamente e pede HOLD. |
| Abertura da garra | Ângulo no ponto 9 do triângulo 4–9–12, calculado pela lei dos cossenos. | Mapeia 60° para 0 (alvo aberto) e 10° para 100 (alvo fechado), limitado a 0–100. Motor 6 é comandado independentemente da IK do braço. |
| Normal da palma | Produto vetorial `(5−0) × (17−0)` nas coordenadas relativas 3D. | É mostrado para diagnóstico; ainda não orienta a garra nem entra no alvo de IK. |

## Como a mão é traduzida para o braço

Quando a terceira palma consecutiva é aceite, o programa lê o braço, calcula por FK a pose atual de `gripper_frame_link` e guarda a posição do pulso e o tamanho 0–9. Nesse frame não envia movimento. Depois, a cada frame válido:

1. Calcula `du = x_atual − x_âncora` e `dv = y_atual − y_âncora` do pulso, mais a variação relativa do tamanho 0–9.
2. Aplica a matriz `IMAGE_YZ_MAP`: horizontal da imagem → Y da base com ganho `+0,50 m/unidade normalizada`; vertical → Z da base com ganho `−0,50 m/unidade normalizada`.
3. Aplica `0,40 m × variação relativa do tamanho 0–9` para obter X da base.
4. Limita cada eixo a ±0,10 m e soma o resultado à pose cartesiana guardada na ativação.
5. Pede ao LeRobot que resolva as cinco juntas do braço em conjunto para alcançar XYZ, preservando aproximadamente a orientação da pose inicial. O peso de orientação usado é 0,01; a posição tem prioridade. Cada comando limita a alteração articular conjunta a uma escala equivalente a `min(0,5 rad/s × intervalo, 1°)` na maior junta.
6. Junta a ação do gripper, se mudou, e envia os alvos `.pos` ao follower LeRobot. O ciclo seguinte volta a ler as posições dos motores.

Assim, o programa calcula uma **translação cartesiana do frame da garra definido pelo URDF**, não a média dos pontos da mão nem uma relação métrica já calibrada entre a mão e a ponta física das mandíbulas. A webcam monocular não fornece profundidade absoluta; por isso, os ganhos atuais são empíricos e 10 cm de movimento da mão não garantem 10 cm da garra.

## O que já observámos e o que falta resolver

| Área | Estado conhecido | Próximo problema a resolver |
|---|---|---|
| Câmara e overlay | `/dev/video0` foi detetada a 640×480 até 30 FPS. A prévia mostra os pontos numerados; a apresentação agora é espelhada. | Confirmar visualmente a nova orientação e medir se o espelho altera a latência de forma relevante. |
| Landmarks/gestos | O código e o overlay atuais classificam PALM, FIST e HAND. Não há rótulo PINCH: a geometria da garra é medida continuamente pelo ângulo 4–9–12. | Os limiares de palma/punho têm testes sintéticos e foram observados em live, mas ainda precisam de validação consistente em diferentes posições da mão. |
| Movimento horizontal | Em teste assistido, o operador confirmou que direita/esquerda passaram a mover-se no sentido esperado. | Medir ganho e alcance; o mapeamento ainda não garante escala física 1:1. |
| Movimento vertical | O operador observou movimento vertical com a mão parada e resposta pouco clara ao mover a mão para cima/baixo. | Encontrar a causa da deriva e estabilizar a referência vertical antes de aceitar XYZ. |
| Profundidade | X usa a escala aparente 0–9. A medida reage à imagem, sem sensor de profundidade. | Separar aproximação real de rotação/perspetiva ou usar uma medida/câmara com profundidade conhecida. |
| Garra | Ângulo geométrico 4–9–12 atualiza o alvo do Motor 6. | Validar fisicamente qual sentido abre/fecha e os ângulos extremos; não confundir o alvo numérico 0–100 com abertura mecânica em percentagem já calibrada. |
| Orientação | A normal da palma aparece no HUD, mas a IK conserva a orientação ancorada e não segue a inclinação da mão. | Só integrar inclinação depois de o XYZ passar validação física; a orientação terá de entrar no mesmo alvo de pose e na mesma chamada de IK. |
| Limites e calibração | LeRobot usa o JSON calibrado para ler e comandar servos. A IK usa a interseção dos limites desse JSON com o URDF. | Rever os limites temporários do URDF para M2/M3 e confirmar a pose inicial e os limites reais sem alterar a calibração inadvertidamente. |
| Desempenho | A medição anterior registou cerca de 40/46 ms p50/p95 na cadeia de visão e 51/57 ms no ciclo completo. MediaPipe corre em CPU. | A meta de 40 ms p95 não foi atingida; ainda é preciso perfilar componentes antes de escolher otimização. |

O teste offline cobre regras e lógica do programa, mas não prova que a câmara, os motores, a geometria real, a escala XYZ ou a segurança mecânica estejam corretos. Houve testes físicos supervisionados em que o braço chegou a ACTIVE e se moveu; o comportamento cartesiano completo continua sem aceitação.

## Repositórios usados e referências

- **Código de controlo atual:** escrito neste projeto; não é um pacote de hand teleoperation copiado de outro repositório.
- **Hugging Face LeRobot 0.7.0:** biblioteca ativa para `SO101Follower`, calibração/conversão de posição, serial Feetech e `RobotKinematics`.
- **Google MediaPipe:** biblioteca ativa e modelo Hand Landmarker para obter os pontos da mão.
- **TheRobotStudio/SO-ARM100:** origem do URDF e das malhas SO-101, adaptados localmente e com proveniência/licença registadas.
- **AFK-Kireeti/Lerobot-SO101, ABMI-software/hand_controlLerobot, sruthihsr/SO101-Gesture-Teleoperation e BrosnanYuen/hand_pose_lerobot_teleop:** consultados como referências. Não são dependências do runtime atual e o código de hand tracking não foi incorporado deles.
- **ROS 2 e NVIDIA Isaac GR00T:** pesquisa histórica; não estão instalados nem participam na execução atual.

Links, commits e atribuições detalhados estão em [`teleop/SOURCES.md`](../teleop/SOURCES.md). A visão geral operacional e os comandos estão no [`README.md`](../README.md).
