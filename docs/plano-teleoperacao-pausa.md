---
title: "Plano: pausa e retomada da teleoperacao"
date: 2026-11-22
type: implementation-plan
status: proposed
---

# Plano: pausa e retomada da teleoperacao

## Goal Capsule

Planejar, sem mudar o controlador agora, uma pausa quando não há mão ou a visão envelhece, seguida de retomada automática quando a mão regressar e estiver estável. Evitar reaplicar indefinidamente o último comando da garra e reancorar mão e robô sem salto de referência. Preservar LeRobot, calibração e a estrutura simples do Brosnan.

Contexto: `Brosnantest/hand_camera_teleop.py`, `Brosnantest/camera_processor.py`, `revisao.md` e `docs/pesquisa-tracking-teleoperacao.md`. Esta é uma proposta; os snippets ilustram o contrato e não foram executados nem estão prontos para copiar sem integrar sincronização e tratar erros.

## Product Contract

- R1. Não modificar código de controlo nesta etapa de planeamento.
- R2. Na perda de mão, na ausência de frames ou numa amostra demasiado antiga, entrar em pausa e impedir a reaplicação do comando anterior da garra.
- R3. Quando a mão voltar e permanecer estável, retomar automaticamente com novas referências da mão e da pose atual do robô. A primeira ação tem deslocamento nulo e comando de garra zero.
- R4. Evitar que imagens ou ações anteriores à pausa sejam usadas depois da retomada.
- R5. Mostrar estados simples: aguardando mão, estabilizando, ativo e pausa/falha. Uma falha de comunicação é distinta de mão ausente.
- R6. Preservar a calibração LeRobot e os limites configurados; não inventar limites de servo nem alegar que a pausa é um corte físico.
- R7. Explicar o ângulo atual, comparar com o ângulo proposto 4–9–12 e conservar a mudança da geometria da garra como decisão separada.
- R8. Ajustar continuamente o sentido horizontal de avanço/recuo à direção dos dedos, sem saltar quando a direção muda; manter o movimento vertical separado. Os eixos e os ganhos do robô devem ser definidos e verificados para a montagem real.

## Planning Contract

### O que existe agora

`Brosnantest/hand_camera_teleop.py:249-256` calcula o ângulo 2D entre `p4 - p0` e `p8 - p0`: do pulso 0 à ponta do polegar 4 e do pulso 0 à ponta do indicador 8. O produto interno normalizado e `arccos` já são usados. A diferença entre ângulos filtrados é enviada como `gripper_vel`; não é um alvo absoluto de abertura. O polegar também contribui para `target_wy`, pelo que mudar a forma da mão pode afetar o punho.

A ideia do operador usa os pontos 4, 9 e 12: vetores `p4 - p9` e `p12 - p9`, com vértice 9 na base do médio. O segmento 4–12 é o lado oposto ao ângulo; é hipotenusa apenas se o triângulo for retângulo. A hipótese é mapear o ângulo ou a distância 4–12 diretamente para a abertura da garra. Isso muda os pontos, a resposta às poses e a semântica de incremento para alvo absoluto. Ambos os ângulos em imagem 2D são afetados por orientação e oclusão; como x e y são normalizados por largura e altura distintas, a geometria deve ser corrigida pela razão de aspeto antes de comparar ângulos em píxeis.

Atualmente qualquer mão detetada inicializa a referência sem esperar estabilidade; não há classificador PALM/FIST/PINCH. O código não exige palma, jacaré ou pinça. Para estudar a garra, uma mão em jacaré, com polegar e indicador visíveis, é uma pose inicial razoável; de perfil os dedos podem ficar ocultos e não existe orientação garantida pelo código. Comparar também palma e pinça em preview antes de escolher. Não afirmar que isso foi validado no braço.

### O que significa estavel

Decisão do operador: reativação **automática após estabilidade**. Proposta inicial, não medida: durante 0,5 s, o ponto 2 atualmente usado deve ficar dentro de uma região de 2% da diagonal da imagem, sem perder a deteção nem receber frames velhos. Utilizar coordenadas corrigidas pela proporção da imagem e comparar toda a janela com um centro robusto, em vez de apenas comparar dois frames adjacentes. Se sair da região, reiniciar a janela. Tempo e tolerância ficam em constantes no início do ficheiro e devem ser ajustados em preview; não são limites de segurança comprovados. A pose precisa também de valores geométricos finitos e vetores não degenerados. Estabilidade da posição não prova que o ângulo da garra ou a escala aparente esteja estável; para armar sem salto, verificar também mudanças dessas grandezas ou iniciar com `gripper_vel=0` e referências recentes, validando qualquer primeiro movimento.

O código atual começa a seguir instantaneamente. Na implementação proposta, a mesma janela de estabilidade será usada no primeiro arranque e na retomada, para não mover ao inserir a mão; o overlay deve indicar que está à espera. Se o operador desejar manter início instantâneo, rever essa decisão antes de executar.

### Estado e decisões técnicas

Propor os estados `WAIT_HAND`, `STABILIZING`, `ACTIVE`, `PAUSED` e `FAULT`. Um relógio monotónico marcado na captura permite ao ciclo do robô detetar dados velhos mesmo se a thread de visão parar. O ponto em que se bloqueia um envio e o ponto em que se muda de estado devem ser sincronizados: uma fila zerada sem protocolo atómico não impede que a thread de visão publique uma ação velha a seguir. Manter uma única propriedade do estado no ciclo do robô e transferir amostras com uma geração/época, ou usar um lock breve com contrato equivalente. Não manter o lock durante inferência ou comunicação serial.

`enabled=False` **não é suficiente por si só**: na fonte LeRobot v0.7.0 `EEReferenceAndDelta` conserva o último alvo desativado e não anula necessariamente a garra. Parar chamadas de escrita também não prova travagem: um servo pode continuar para o objetivo já recebido. Em pausa, com leitura de juntas recente e comunicação funcional, estudar e verificar um alvo de manutenção das posições lidas e `gripper_vel=0`, enviando-o com o LeRobot e sem converter manualmente calibração. Se uma leitura/escrita falhar, assinalar FAULT e não prometer que qualquer HOLD ou desativação de torque chegou aos motores. A política de parada e o comportamento sob carga precisam de verificação supervisionada.

### Sentido da mao e referenciais

**Esclarecimento de 22 de novembro:** o operador quer o espaço matemático do robô fixo à sua base. Ao iniciar o script, o programa deve ler as posições dos motores e calcular, por cinemática direta, a pose inicial estimada do referencial da garra nesse espaço. Isso é estimativa a partir das juntas e do URDF, não medição externa do XYZ de cada motor. Enquanto a ligação estiver válida, manter a leitura das juntas atualizada. Na primeira ativação e em cada retomada após pausa, associar uma leitura recente à referência da mão; atualizar a referência da garra não desloca a origem nem roda o espaço fixo da base. Se a leitura inicial ou posterior for inválida, não fingir que a pose é conhecida.

O operador escolheu acompanhar continuamente a direção para onde os dedos apontam. No código atual, `target_x = Δx` da imagem, `target_z = −Δy` da imagem e `target_y = −Δescala`: ver `Brosnantest/hand_camera_teleop.py:303-310`. A posição é a do ponto 2, na base do polegar; a escala é uma estimativa 2D. O `target_x` é um comando cartesiano que entra em `EEReferenceAndDelta`, não o ângulo do motor 1 nem necessariamente a direção direita para o observador. A IK calcula conjuntamente os alvos das juntas para o `gripper_frame_link`; a direção desse X é a do modelo/referencial do robô, não uma direção inferida automaticamente pela webcam.

#### Seta central: hipoteses para comparar

O operador imagina uma seta que flutua entre o polegar e o médio e continua a apontar para a frente ao abrir e fechar a mão. A direção e o ponto em que a seta é desenhada podem ser calculados separadamente: desenhar a seta no ponto médio das pontas não obriga a usar essas pontas para decidir a direção.

1. **Bissetriz visual candidata:** usar `a = p4 − p9` e `b = p12 − p9`, normalizar cada um e calcular `d = normalize(normalize(a) + normalize(b))`. Representa a direção entre os dois vetores, mas **não é invariável à abertura**: se apenas o polegar se mover, o ângulo central muda; pontas ocluídas e vetores opostos ou curtos também causam instabilidade. Desenhar na posição `m = (p4+p12)/2` não altera esta propriedade. Não assumir que o vetor corresponde à direção desejada sem observar o gesto.
2. **Candidato baseado na palma:** `c = (p5 + p9 + p13 + p17)/4`, `d_palma = normalize(c − p0)`. Pode ser mais estável ao dobrar os dedos, pois usa as suas bases e o pulso, mas aproxima o eixo longitudinal da palma, não garante que seja a bissetriz visual das pontas nem que a direção fique exatamente invariável. Os pontos também são estimados e podem oscilar ou ficar ocluídos. Outro candidato simples é `p9 − p0`.
3. **Teste de comparação, sem escolha antecipada:** observar ambas as setas enquanto a palma fica imóvel e a garra humana abre e fecha; registar variação angular, perda de pontos, movimentos falsos e comportamento de frente/de lado. Para o controlo inicial, favorecer a mais estável entre as que correspondam à intenção do operador. Uma seta pode ser mostrada no espaço entre os dedos mas ter a sua direção calculada pela palma; indicar visualmente qual variante está a ser avaliada.

#### Comparar A, B e C: plano de visualizacao

O operador propôs ajustar uma reta aos pontos **5, 9, 13 e 17**, que não estão necessariamente alinhados, e desenhar uma normal no centro. A operação é um **ajuste ortogonal por componentes principais (PCA)**: a reta passa pelo centro dos quatro pontos e minimiza a soma dos quadrados das distâncias perpendiculares a ela. É diferente de forçar a reta a passar exatamente por todos os pontos. A decomposição por valores singulares, SVD, fornece o eixo de maior variação sem problemas especiais quando a reta é vertical ([NumPy: SVD](https://numpy.org/doc/stable/reference/generated/numpy.linalg.svd.html)).

| Seta | Direção a comparar | Limitação |
|---|---|---|
| A | Pulso 0 → base do médio 9 | É longitudinal à palma; o pulso ou a base do dedo podem mover-se ou ficar ocultos. |
| B | Normal em imagem à reta PCA das bases 5, 9, 13 e 17, desenhada no centro | A linha tem duas normais; usa-se o pulso 0 para escolher a que aponta para fora da palma. Se a mão estiver de lado, os pontos podem sobrepor-se ou a direção ficar ambígua. |
| C | Bissetriz dos vetores 9→4 e 9→12 | Pode girar quando apenas um dedo abre ou fecha e pode desaparecer por oclusão. |

A normal B é perpendicular **à reta 2D na imagem**; não é a normal 3D do plano da palma, nem garante que os dedos apontem sempre nessa direção. A mão pode ser curva ou os pontos ter ruído. Os valores de confiança e tolerância ficam por medir em preview; não converter uma hipótese matemática num comando de motor sem decisão e validação.

**Onde inserir:** em `Brosnantest/hand_camera_teleop.py`, perto de `to_vec()` (`:132`), criar uma pequena função que receba landmarks e dimensões do frame e devolva centro/direção válida ou `None`. Dentro de `frame_display()`, depois de desenhar os pontos (`:222-234`), desenhar as setas A/B/C com rótulos e cores diferentes. Isto é uma proposta para implementação posterior. O script atual abre a serial e liga o braço durante o arranque, pelo que o trabalho de implementação deve primeiro providenciar uma opção de **preview sem ligação ao robô**; não executar o script atual como se fosse preview.

Exemplo proposto para a seta **B**, com parâmetros ilustrativos a calibrar em imagens; não é código implementado:

```python
def palm_axis_b(landmarks, width, height, min_spread_px, min_axis_gap, min_wrist_px):
    scale = np.array([width, height], dtype=float)
    points = np.array([to_vec(landmarks[i]) * scale for i in (5, 9, 13, 17)])
    wrist = to_vec(landmarks[0]) * scale
    if not np.isfinite(points).all() or not np.isfinite(wrist).all():
        return None

    center = points.mean(axis=0)
    _, values, vh = np.linalg.svd(points - center, full_matrices=False)
    if values[0] <= min_spread_px or (values[0] - values[1]) / values[0] < min_axis_gap:
        return None  # pontos demasiado juntos ou sem direção dominante

    tangent = vh[0]
    normal = np.array([-tangent[1], tangent[0]])
    away_from_wrist = np.dot(normal, center - wrist)
    if abs(away_from_wrist) < min_wrist_px:
        return None  # o pulso não permite escolher o sentido com confiança
    if away_from_wrist < 0:
        normal = -normal
    return center, normal
```

`min_spread_px`, `min_axis_gap` e `min_wrist_px` são nomes de parâmetros sem valores aprovados; `values` são valores singulares, não distâncias diretas de um ponto individual. Tratar também eventual `np.linalg.LinAlgError` como medição inválida. Se B for inválida, esconder ou indicar `B indisponível`, não apresentar uma seta antiga como atual. A orientação é calculada após multiplicar coordenadas normalizadas pela largura e altura, preservando o aspeto em píxeis.

Exemplo visual a inserir no bloco de desenho, após o cálculo das três variantes:

```python
for label, candidate, color in [("A", arrow_a, (0, 255, 0)),
                                ("B", arrow_b, (255, 0, 0)),
                                ("C", arrow_c, (0, 255, 255))]:
    if candidate is None:
        continue
    center, direction = candidate
    start = tuple(np.rint(center).astype(int))
    end = tuple(np.rint(center + 60 * direction).astype(int))
    cv2.arrowedLine(image, start, end, color, 2, tipLength=0.2)
    cv2.putText(image, label, end, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
```

`arrow_a`, `arrow_b` e `arrow_c` representam pares `(centro, direção unitária)` a calcular de acordo com a tabela; 60 píxeis é apenas comprimento de desenho. A e C também precisam de rejeitar normas demasiado pequenas e pontos inválidos; C deve rejeitar vetores quase opostos antes de normalizar a soma. Podem usar um centro gráfico comum para comparar orientação, desde que a origem matemática de cada vetor continue correta. Separar o desenho dos comandos: nenhuma seta A/B/C deve ser ligada à ação do robô na fase de comparação.

Para qualquer vetor em imagem, converter primeiro os pontos normalizados para píxeis usando `u = x × largura` e `v = y × altura`; normalizar só após essa conversão para não distorcer os ângulos pela razão largura/altura. Validar normas e valores finitos antes da divisão. As coordenadas são estimativas sujeitas a oclusão e perspetiva. A [documentação MediaPipe](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker) distingue coordenadas de imagem dos landmarks 3D estimados centrados na mão; nenhum deles, por si só, mede a posição global da mão em relação à base do braço.

#### Mapeamento para o espaco fixo

Proposta mínima para o caso horizontal descrito: obter a componente horizontal da seta escolhida, em píxeis. Se aponta claramente para a esquerda da imagem, deslocar a mão para a esquerda deve contribuir positivamente para o avanço lógico; se aponta claramente para a direita, deslocar para a direita deve contribuir positivamente. Movimento vertical continua a ser calculado como `−Δy_imagem` e enviado ao eixo vertical do modelo depois da correspondência definida para a montagem. Essa separação evita que a inclinação no plano misture acidentalmente subir/descer com avanço.

```python
# Exemplo direcional, ainda nao implementado; seta_d é candidata e já está em pixeis.
direction = +1 if seta_d.x > horizontal_threshold else -1 if seta_d.x < -horizontal_threshold else None
delta_forward = direction * (hand_x_now - hand_x_prev) if direction is not None else None
```

Assim, mão apontada para a esquerda (`direction=-1`) e movimento para a esquerda (`hand_x_now-hand_x_prev < 0`) dão `delta_forward > 0`. A tolerância horizontal é um parâmetro **a medir**, não um limiar seguro definido neste plano. Seguir a orientação continuamente não deve multiplicar todo o deslocamento desde a referência por um sinal que muda: isso inverteria o alvo inteiro e causaria um salto. Em vez disso, integrar incrementalmente os movimentos aceites, conservando o alvo acumulado quando muda a direção e zerando o incremento na transição. Decidir o sinal com histerese, filtrar a orientação e rejeitar saltos de landmarks; se o vetor for demasiado curto, quase vertical, instável, ou os pontos estiverem ocluídos, pausar e invalidar o incremento, sem atribuir sentido arbitrário. Ao recuperar, reancorar com os procedimentos de U2/U3.

Por exemplo, depois da reancoragem numa posição `X_ref` do espaço fixo do robô, a forma conceptual é `X_alvo(k) = X_ref + ganho × soma(delta_forward válido)`. O sinal da seta afeta os **novos incrementos**, não o sistema de coordenadas do robô nem o acumulado anterior. Com o ganho constante, movimento da mão de 10 unidades para a esquerda enquanto a seta aponta à esquerda soma 10 unidades lógicas antes de aplicar o ganho; 3 unidades para a direita subtraem 3. Esta ilustração não diz que as unidades de imagem são centímetros. Confirmar se `target_x` da cadeia instalada é delta face à referência e incluir o fator `end_effector_step_sizes` existente uma única vez; não somar `X_ref` duas vezes. IK e limites operam sobre o alvo na base fixa.

Esta regra é uma convenção **em duas dimensões**: não determina automaticamente o que é frente real em 3D, a orientação da câmara relativamente à base, ou como o braço está virado. Uma correspondência câmara→base deve ser escolhida e verificada por eixo na montagem real. A D435i pode ajudar futuramente com translação 3D, mas não substitui essa escolha. Quando a mão gira, o próprio ponto 2 pode deslocar-se e gerar falsa translação; comparar em preview com um centro de pontos da palma. Primeiro manter o mapeamento numa avaliação sem motores; após a estabilidade e pausa funcionarem, testar só uma direção de cada vez sob supervisão. A visualização pode estar espelhada; definir e testar o sinal sobre as mesmas coordenadas utilizadas pela lógica.

### Fora de escopo

Integração D435i, novo modelo YOLO, nova cinemática, substituir LeRobot ou recalibrar motores; tornar o gesto da garra absoluto; otimização de desempenho ampla; afirmar que o sistema é funcionalmente seguro. A geometria alternativa é pesquisada e especificada abaixo como opção futura, não ligada aos motores automaticamente.

## Implementation Units

### U1. Carimbar captura e manter somente dados atuais

**Arquivo:** `Brosnantest/hand_camera_teleop.py`. Associar a cada frame o tempo monotónico de captura e uma identificação de geração. Substituir frames e ações pendentes por dados mais recentes; evitar ler `empty()` e depois assumir que uma operação separada é atómica. Preservar a distinção entre frame sem mão e ausência de frame. O watchdog deve ser verificado na thread do robô, não depender exclusivamente de uma thread de visão que pode parar.

Esboço do contrato, a adaptar à sincronização escolhida:

```python
sample = {"captured_at": time.monotonic(), "epoch": epoch, "frame": frame}
# A publicação de sample substitui o frame pendente sob o mesmo protocolo de sincronização.
```

O timestamp acima deve ser registado imediatamente após `cap.read()` devolver; é um aproximador da captura, não necessariamente o instante de exposição no sensor. Ajustar o contrato ao driver quando houver timestamp de hardware. Determinar o prazo `MAX_FRAME_AGE` a partir de medições; até lá manter explicitamente configurável, sem interpretar uma sugestão numérica como validada.

### U2. Perda, pausa e garra

**Arquivo:** `Brosnantest/hand_camera_teleop.py`. Um frame processado sem mão marca perda; ausência de resultados ou captura desencadeia watchdog no robô. Em ambos, invalidar ações pendentes e a época corrente. O comando anterior da garra não pode continuar a ser reaplicado nem durante a operação normal quando não chega uma nova medição: `gripper_vel` representa um incremento e deve ser consumido uma vez por ação. O alvo cartesiano e o da garra têm semânticas diferentes; descartar frames intermédios pode alterar o total de incrementos. Tratar isso explicitamente ou calcular a diferença desde a última medição realmente consumida. Não basta trocar a fila por tamanho 1.

```python
# Esquema na thread proprietária do robô; integrá-lo num protocolo atómico.
if no_hand or now - last_valid_capture > MAX_FRAME_AGE:
    state = "PAUSED"
    last_gripper_delta = 0.0
    invalidate_pending_actions_and_epoch()
    hold_current_measured_joints_if_readback_is_fresh()
```

As funções com nomes descritivos são pseudocódigo de contrato, não chamadas já existentes. A transição deve impedir novos objetivos vindos da visão após pausa, inclusive quando um `send_action` já começou antes da sinalização; documentar e limitar a janela de corrida. Um comando em trânsito não pode ser magicamente cancelado. Se readback estiver inválido, não enviar um HOLD inventado; entrar em FAULT, avisar e seguir a política de erro a validar.

### U3. Estabilização e reancoragem automática

**Arquivo:** `Brosnantest/hand_camera_teleop.py`. Após perda, nova mão válida entra em `STABILIZING`; manter janela temporal conforme a definição acima. A transição para `ACTIVE` deve ser coordenada pela thread do robô: usar amostra recente da mão e leitura recente das juntas; descartar ações da época anterior; reiniciar filtros e `prev_gripper_angle`; reiniciar a referência do processador LeRobot e iniciar com ação neutra. Confirmar no código instalado a API `reset()` e a ordem real de `EEReferenceAndDelta`: a reancoragem da mão sem reancorar a FK do robô não basta.

```python
if state == "STABILIZING" and hand_is_stable(samples) and fresh(robot_obs):
    invalidate_pending_actions_and_epoch()
    hand_reference = latest_hand_sample
    reset_vision_filters_and_previous_gripper_angle(hand_reference)
    reset_lerobot_reference()  # conferir a API e executá-la na thread do robô
    camera_action = neutral_action(gripper_vel=0.0)
    state = "ACTIVE"
```

Os passos não podem ser executados desordenadamente entre threads; `latest_hand_sample` deve corresponder à nova época e ter idade aceitável. A transição inicial deve usar a mesma lógica. Se a deteção oscilar, regressar a `PAUSED` e reiniciar a janela. Retomada automática após estabilidade é decisão do operador; pode ocorrer sem novo gesto deliberado e deve ficar visível como risco operacional.

### U4. Mostrar estado e registar diagnóstico mínimo

**Arquivo:** `Brosnantest/hand_camera_teleop.py`. No feed: estado e indicação simples de estabilidade, sem sobrepor um painel. No terminal: apenas transições e falhas, com tempo da última amostra válida. Não imprimir a cada frame nem inventar percentagens de confiança de gestos. `Esc` continua como saída normal.

### U5. Avaliar geometria da garra sem alterar o controlo físico

Usar um modo de observação ou gravações previamente autorizadas, sem abrir serial ou enviar ações, para visualizar ângulo atual e proposto. O projeto atual liga o braço no arranque; não declarar que esse modo de preview já existe. Comparar palma, jacaré aberto/fechado e pinça, frente e lado, observando perda/oclusão, oscilações e continuidade. Tratar normas próximas de zero e valores não finitos. Eventual mapeamento absoluto para a faixa calibrada da garra depende de uma decisão posterior; limites humanos 10°/60° são hipóteses e não limites do motor.

```python
# Geometria proposta, a avaliar sem acionamento; p contém coordenadas coerentes em píxeis.
a, b = p[4] - p[9], p[12] - p[9]
cos_theta = np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1)
theta_deg = np.degrees(np.arccos(cos_theta))
```

Antes da divisão, verificar que ambos os vetores têm norma acima de tolerância e que os valores são finitos; caso contrário, sinalizar medição inválida, não produzir comando.

### U6. Adaptar continuamente o sentido horizontal da mao

**Arquivo futuro:** `Brosnantest/hand_camera_teleop.py`. Integrar a regra da secção “Sentido da mao e referenciais” depois da gestão de pausa de U1–U3; recolher a direção em cada frame aceite e aplicar apenas aos deslocamentos incrementais posteriores. Manter subida e descida independentes, invalidar direção ambígua com histerese e preservar a continuidade do alvo na inversão. Reancorar posição, direção e alvo na mesma transição já prevista de pausa/retomada. Não mudar o URDF nem a calibração e não aplicar uma rotação da orientação da mão à orientação da garra sem decisão separada.

Comparar a bissetriz 4–9–12 com a direção da palma numa visualização sem motores e registar a escolha junto do operador antes de ligar qualquer variante ao movimento. A direção e a posição visual da seta são variáveis distintas. O mapeamento horizontal acima é uma primeira aproximação ao pedido, não demonstra seguimento real tridimensional da direção do dedo. A integração com depth e um vetor tridimensional fica fora deste plano até haver medição e escolha explícita de correspondência de eixos.

Incluir a candidata B como alternativa principal pedida pelo operador e manter A/C para comparação; verificar frente, perfil, palma aberta, jacaré, dedos ocluídos, pulso perto da reta e pontos quase coincidentes. Escolher somente depois do preview e do acordo com o operador.

A numeração dos 21 pontos já foi acrescentada ao desenho na janela para o operador poder identificar landmarks; é uma alteração visual e não implementa a adaptação dos eixos. Avaliar legibilidade e latência em futura visualização, sem considerar isso prova de controlo físico.

## Verification Contract

Para futura implementação, os cenários devem ser realizados no container do projeto, primeiro com dados simulados ou gravados, sem motores. A presente etapa é somente planeamento; não executar testes agora.

1. Sem mão na inicialização: nenhum objetivo de movimento criado; mostrar espera.
2. Mão aparece mas oscila além da tolerância: ainda não ativa; mão permanece na região por 0,5 s com dados válidos: a transição ativa é única.
3. Antes do primeiro movimento, confirmar mão e FK do braço ancoradas na mesma transição, ação de translação zero e garra zero, sem salto de referência.
4. Perda de mão, falha de captura, congelamento da thread de visão e frames velhos: estado pausa; zero reaplicação de delta anterior; política de HOLD baseada em readback e resultados observados.
5. Filas cheias, alternância rápida, timestamps fora de ordem e mensagens de época antiga: nenhuma ação antiga é aceite depois da pausa. Ensaiar o caso de uma escrita em trânsito e reportar o limite real.
6. Mão retorna deslocada, imóvel pelo intervalo: reancora com leitura recente e sem saltar para a posição antiga; saídas/reentradas repetidas não reutilizam filtros ou delta anteriores.
7. Garra: um delta é consumido no máximo uma vez quando a visão é mais lenta que o ciclo do robô; quando há substituição de amostras, o sinal não perde nem reaplica movimento sem análise.
8. Readback ausente, exceção da câmara/IK/serial e encerramento: avisar, distinguir FAULT, não afirmar que o braço parou sem comprovação. Rever ciclo de vida das threads e limpeza.
9. Cenário de regressão: eixos, resolução, filtros e LeRobot existentes são mantidos exceto alterações explicitamente acordadas; referências a resultados físicos só após ensaio.
10. Apenas depois dos cenários sem motores, ensaio físico supervisionado com braço apoiado, área livre e corte acessível: observar posição ao iniciar, pausar, retirar/reinserir mão e garra. Confirmar o comportamento real, não apenas logs ou sucesso de escrita.
11. Testar lógica de sentido com a mão apontada à esquerda: deslocar para a esquerda deve somar no avanço lógico e para a direita subtrair; repetir com mão apontada à direita e sinais opostos. Subir e descer não podem inverter com a orientação horizontal.
12. Inverter a orientação sem transladar: o alvo não pode saltar. Testar rotação progressiva, vetor quase vertical, oclusão, ruído perto do limiar, mudança de mão e espelhamento. Enquanto a direção é ambígua, não gerar novo deslocamento; ao voltar a uma direção válida, reiniciar o incremento e seguir a política de pausa/reancoragem.
13. Verificar no modelo e, depois, em ensaio supervisionado, a correspondência entre avanço lógico, o eixo base do robô e a direção física pretendida; detetar separadamente alteração do ponto 2 provocada apenas por rotação da mão.
14. Com a palma imóvel, abrir e fechar a mão: comparar quanto variam as setas da bissetriz e da palma; não assumir independência perfeita. Verificar dedos sobrepostos e pontos ausentes.
15. Na inicialização, verificar leitura de todas as juntas e FK inicial; na retomada verificar leitura recente e FK nova no mesmo referencial fixo da base. Perda de serial impede declarar pose atual conhecida.

## Definition of Done

- A escolha de retomada automática e a definição de estabilidade são aprovadas e documentadas; parâmetros calibrados por observação.
- Evidência dos cenários acima, incluindo watchdog, erro serial e ausência de reenvio de delta da garra. Os resultados não substituem um teste físico.
- Sem regressão no acesso ao modelo, calibração, câmara e processadores do projeto. Documentação atualizada com limitações, principalmente que HOLD e corte físico são coisas diferentes.
- Qualquer alteração dos pontos de garra ou troca para D435i é objeto de decisão própria.

## Appendix

Referência da implementação atualmente usada: [`Brosnantest/hand_camera_teleop.py`](../Brosnantest/hand_camera_teleop.py). A lógica de `EEReferenceAndDelta`/`GripperVelocityToJoint` deve ser conferida no ambiente instalado; referência pública [LeRobot v0.7.0](https://github.com/huggingface/lerobot/blob/v0.7.0/src/lerobot/robots/so_follower/robot_kinematic_processor.py).
