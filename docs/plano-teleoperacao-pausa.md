---
title: "Plano mínimo: direção da base e rotação do punho"
date: 2026-11-23
type: implementation-plan
status: proposed
---

# Plano mínimo: M1 e M5 controlados pela mão

Este é o plano revisto para aprovação. O objetivo é alterar o mínimo do controlador que já funciona: a seta B define M1 e a linha 5–17 define M5. M2–M4 continuam a ser calculados pela IK existente; M6 continua com o controlo atual da pinça. Não aplicar este plano ao código de teleoperação nem iniciar o braço antes da aprovação.

## O que fica no âmbito

- Usar `Brosnantest/hand_camera_teleop.py`, a câmara, MediaPipe, as filas e a cadeia LeRobot atuais.
- Acrescentar a seta B e usar a sua mudança de direção para propor o ângulo de **M1** (`shoulder_pan`, base).
- Usar a mudança de ângulo da linha entre os pontos **5 e 17** para propor o ângulo de **M5** (`wrist_roll`, punho).
- Fixar esses dois ângulos durante a IK e deixar M2–M4 livres para calcular o alvo atual. Continuar a usar os motores e a calibração através do LeRobot.
- Manter a translação atual baseada no ponto 2 e o cálculo atual da garra M6. Os números dos landmarks já aparecem no vídeo.
- Acrescentar pausa efetiva ao perder a mão e retomada automática na primeira deteção válida após o regresso, sem exigir imobilidade.

Uma nova geometria de pinça, outras setas, nova cinemática de arco, novos limites de movimento, calibração de câmara e RealSense ficam fora desta alteração. Não remover proteções existentes. Até a pausa proposta ser implementada, o comportamento atual ao perder a mão continua a conservar a última ação.

## Filas e limites que já existem

Uma **fila** é uma lista temporária entre partes do programa: `frame_queue` passa imagens da câmara à visão; `action_queue` passa os movimentos calculados à thread que comunica com o braço. Ambas já existem. A mudança necessária para a pausa é identificar e descartar ações de uma deteção anterior, para que não sejam enviadas depois da perda da mão. Não é necessário criar outra fila ou alterar a capacidade das duas.

No código atual, `EEBoundsAndSafety` limita as coordenadas cartesianas entre −1 e +1 m e verifica passos acima de 0,10 m; esses valores não são limites individuais dos motores. A garra é limitada pelo processador LeRobot a 0–100. `max_relative_target` não foi configurado e tem valor padrão `None`, logo não está ativo um limite adicional por passo de motor no `send_action`. Vamos preservar a cadeia e a calibração existentes, sem introduzir novos limites de movimento. **Atenção:** a máscara da IK fixa M1/M5 nos valores escolhidos; não se deve afirmar que ela verifica automaticamente os limites físicos desses valores.

## Antes de alterar: como separar os motores

A versão instalada do LeRobot usa `RobotKinematics` com o PlaCo. O PlaCo já dispõe de `mask_dof` para impedir que a IK altere uma junta enquanto resolve as restantes:

```python
kinematics_solver.solver.mask_dof("shoulder_pan")  # M1
kinematics_solver.solver.mask_dof("wrist_roll")    # M5
```

O detalhe é que o LeRobot repõe as posições a partir do vetor passado à IK em cada chamada. Logo, os alvos de M1 e M5 devem estar nesse vetor **antes** de calcular M2–M4. Apenas trocar os dois valores depois da IK daria a M2–M4 uma solução calculada para outra posição da base e do punho.

**Alteração mínima candidata:** uma adaptação local de `RobotKinematics.inverse_kinematics()` recebe uma cópia do vetor de juntas, coloca nela os dois alvos da mão e chama a implementação existente do LeRobot. A observação real continua a ser usada pelos restantes processadores. Os alvos são atualizados na thread do robô antes da chamada à cadeia, usando a amostra da mão recebida. Resolver índices por nome dos motores, evitando presumir a ordem.

```python
# Esquema do encaixe, ainda não é patch completo.
q_para_ik = current_joint_pos.copy()
q_para_ik[indice_m1] = alvo_m1_graus
q_para_ik[indice_m5] = alvo_m5_graus
return super().inverse_kinematics(q_para_ik, desired_ee_pose, ...)
```

A máscara só impede a IK de alterar M1/M5; os alvos continuam a mudar quando chegam novas medições. Os restantes cálculos consideram os valores colocados no vetor. Se não houver uma medição inicial válida da mão, utilizar a leitura atual como referência dos dois motores.

### Código candidato para a IK

Em `Brosnantest/hand_camera_teleop.py`, colocar esta classe depois dos imports e antes de `robot_config`; na criação do solver substituir `RobotKinematics(` por `FixedJointKinematics(`. Os `mask_dof` acima ficam dentro da classe e não devem ser duplicados.

```python
class FixedJointKinematics(RobotKinematics):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fixed_targets = None
        self.base_index = self.joint_names.index("shoulder_pan")
        self.roll_index = self.joint_names.index("wrist_roll")
        self.solver.mask_dof("shoulder_pan")
        self.solver.mask_dof("wrist_roll")

    def inverse_kinematics(self, current_joint_pos, desired_ee_pose, *args, **kwargs):
        joints = np.array(current_joint_pos, dtype=float, copy=True)
        if self.fixed_targets is not None:
            joints[self.base_index], joints[self.roll_index] = self.fixed_targets
        return super().inverse_kinematics(joints, desired_ee_pose, *args, **kwargs)
```

#### Respeitar a calibração já existente

O LeRobot instalado não recorta automaticamente os comandos em graus pelos extremos da calibração, e a máscara fixa a junta no valor recebido. Assim, verificar os dois alvos contra os limites **já registados**, sem criar novos valores. Colocar estas funções perto da classe anterior; após `robot.connect()` e antes de iniciar as threads, executar `fixed_bounds = calibrated_fixed_bounds(robot.bus)`. `_normalize` é método privado da versão instalada: se a API falhar, terminar com erro em vez de continuar sem verificação.

```python
def calibrated_fixed_bounds(bus):
    bounds = {}
    for name in ("shoulder_pan", "wrist_roll"):
        motor = bus.motors[name]
        cal = bus.calibration[name]
        if cal.id != motor.id or cal.range_min >= cal.range_max:
            raise ValueError(f"calibração inválida: {name}")
        low = float(bus._normalize({motor.id: cal.range_min})[motor.id])
        high = float(bus._normalize({motor.id: cal.range_max})[motor.id])
        if not np.isfinite([low, high]).all() or low >= high:
            raise ValueError(f"limites calibrados inválidos: {name}")
        bounds[name] = (low, high)
    return bounds

def fixed_targets_in_bounds(targets, bounds):
    return all(np.isfinite(value) and bounds[name][0] <= value <= bounds[name][1]
               for name, value in zip(("shoulder_pan", "wrist_roll"), targets))
```

No consumidor, construir primeiro os dois valores numa variável `targets`; verificar a leitura atual e depois os alvos com `fixed_targets_in_bounds`. Caso falhem, parar e mostrar o motor/leitura para diagnóstico; não comandar automaticamente um movimento para dentro da faixa. Só após passar, atribuir `kinematics_solver.fixed_targets = targets`. A calibração de M5 pode cobrir uma volta completa e não confirma a liberdade física dos cabos. Esta verificação não garante alcance da IK, colisões ou paragem física.

A thread do robô define `kinematics_solver.fixed_targets = (alvo_m1_graus, alvo_m5_graus)` antes da cadeia. A versão instalada do processador LeRobot monta o vetor pela ordem das posições no dicionário da observação; é necessário verificar que essa ordem corresponde a `robot.bus.motors` antes de o usar. Esta classe preserva a leitura real para a FK de referência. A integração com a pausa é descrita abaixo; não aplicar este trecho isoladamente.

## Sinais da mão

1. **Seta B:** usar os pontos 5, 9, 13 e 17 para estimar a linha da base dos dedos; desenhar a perpendicular B na imagem. Registar a direção da mão e a posição de M1 na primeira deteção. A mudança angular de B gera o objetivo relativo de M1, com sinal e escala a conferir no teste.
2. **Linha 5–17:** registar a sua orientação e a posição de M5 na mesma referência. A mudança angular da linha gera o objetivo relativo de M5, com sinal e escala a conferir no teste.
3. O controlo anterior `target_wy`, calculado a partir do polegar e enviado à IK como orientação, deve deixar de acrescentar uma segunda ordem de rotação ao mesmo gesto. Manter a translação e a garra como estão.

**Limitação a observar:** se B for exatamente a perpendicular da linha 5–17 e ambos os ângulos forem medidos na mesma imagem 2D, giram juntos. Isso pode fazer M1 e M5 responderem ao mesmo movimento. Começar pelo cálculo simples pedido e verificar o comportamento; se impedir o controlo pretendido, discutir um sinal independente antes de complicar a implementação. A imagem 2D também não mede diretamente a rotação horizontal física da mão em todas as posições.

### Código candidato para a seta e a linha

No mesmo ficheiro, colocar depois de `to_vec()`. `cv2.fitLine` já faz o ajuste de uma linha aos quatro pontos; píxeis evitam distorcer os ângulos pela proporção 640×480. A margem de um píxel evita dividir uma linha sem comprimento; não é limite de movimento.

```python
def hand_geometry(landmarks, width, height):
    points = np.array([[landmarks[i].x * width, landmarks[i].y * height]
                       for i in (5, 9, 13, 17)], dtype=np.float32)
    wrist = np.array([landmarks[0].x * width, landmarks[0].y * height])
    if not np.isfinite(points).all() or not np.isfinite(wrist).all():
        return None
    span = points[-1] - points[0]
    if np.linalg.norm(span) < 1.0:
        return None
    vx, vy, cx, cy = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    line = np.array([vx, vy])
    if np.dot(line, span) < 0:
        line = -line
    center = np.array([cx, cy])
    forward = np.array([-line[1], line[0]])
    projection = np.dot(forward, center - wrist)
    if abs(projection) < 1.0:
        return None
    if projection < 0:
        forward = -forward
    return (float(np.degrees(np.arctan2(forward[1], forward[0]))),
            float(np.degrees(np.arctan2(span[1], span[0]))), center, forward)

def angle_delta(angle, reference):
    return (angle - reference + 180.0) % 360.0 - 180.0
```

Em `frame_display()`, depois de definir `width` e `height`, chamar `hand_geometry(hand_landmarks, width, height)`. Se o resultado for `None`, tratar a mão como não disponível. Caso contrário, desempacotar `b_angle, roll_angle, center, forward`, desenhar B e enviar os dois ângulos com a ação. `angle_delta` transforma a mudança de ângulo da mão em graus relativos à referência; o sentido e a escala iniciam em 1:1 e serão observados pelo operador.

```python
cv2.arrowedLine(image, tuple(center.astype(int)),
                tuple((center + 50 * forward).astype(int)), (0, 0, 255), 2)
```

## Pausa e retomada

A pausa é parte do plano. A primeira deteção válida inicia o controlo; depois de uma perda, a primeira deteção válida após o regresso estabelece imediatamente novas referências. Não há espera por estabilidade.

- **Perdeu a mão ou a visão deixou de produzir amostras recentes:** entrar em `PAUSED`, informar no vídeo/terminal, invalidar ações antigas e deixar de enviar o último incremento da garra. Com uma leitura recente dos motores, pedir uma única vez que mantenham as posições lidas; se leitura ou escrita falhar, mostrar erro e não afirmar que o braço parou.
- **Sem amostra nova:** não recalcular nem reenviar repetidamente a última ação. Cada amostra válida é consumida no máximo uma vez; se não chegar outra dentro do intervalo definido, entrar em pausa.
- **Mão regressou:** obter uma leitura recente do braço e criar imediatamente novas referências para a mão, posição atual, M1, M5 e garra. Reiniciar os filtros necessários, preservando as proteções existentes. A primeira amostra serve de referência: regressar noutro ponto do quadro não deve gerar um salto.
- **Concorrência:** a perda de mão deve invalidar também uma ação que ainda esteja a ser calculada. Só aceitar o estado dessa ação depois de confirmar que a amostra continua válida e o envio ocorreu; não manter uma referência a um passo antigo após substituir a cadeia. Um comando já em transmissão não pode ser cancelado por esta lógica.

### Contrato mínimo entre as threads

Não trocar as filas existentes. A fila de imagens precisa carregar também o instante da captura, para distinguir uma imagem recente de uma imagem antiga: alterar a linha `frame_queue.put(frame)` de `frame_capture()` para `frame_queue.put((time.monotonic(), frame))`, e em `frame_display()` substituir `frame = frame_queue.get()` por `captured_at, frame = frame_queue.get()`. Esse instante é registado ao terminar `cap.read()`, não mede atraso já existente no driver da câmara.

Acrescentar `import copy` junto dos imports; depois de `action_queue = queue.Queue(maxsize=100)` acrescentar:

```python
tracking_lock = threading.Lock()
tracking_state = {"visible": False, "epoch": 0, "captured_at": 0.0, "status": "PAUSED"}
MAX_FRAME_AGE = 0.5  # segundos sem imagem recente antes da pausa; ajustar no teste
```

`MAX_FRAME_AGE` é um prazo para detetar câmara parada, não um limite de movimento ou uma espera de estabilidade. Sem algum prazo, não existe no código atual um sinal que permita distinguir uma câmara parada de um intervalo normal entre frames. Na primeira amostra válida depois de `visible=False`, incrementar `epoch` e reiniciar os filtros `hand_pos`, `hand_depth`, `gripper_angle` e as referências da mão. Quando a mão não estiver presente ou `hand_geometry` retornar `None`, colocar `visible=False` e incrementar `epoch` uma vez na transição. Todas as leituras e escritas de `tracking_state` usam `tracking_lock`.

No bloco de ação de `frame_display`, enviar **valores filtrados absolutos**, para que o consumidor possa descartar amostras antigas sem perder deltas de M6. Guardar uma cópia de `hand_pos`; os ângulos estão em graus. Substituir o dicionário atualmente colocado na `action_queue` por:

```python
action = {
    "epoch": epoch,
    "captured_at": captured_at,
    "hand_pos": hand_pos.copy(),
    "hand_depth": float(hand_depth),
    "b_angle": b_angle,
    "roll_angle": roll_angle,
    "gripper_angle": float(gripper_angle),
}
```

O produtor só publica a ação se a captura ainda tiver idade válida. O consumidor retira a amostra mais recente e rejeita epochs anteriores. Ao reentrar, usa a amostra e uma leitura do braço no mesmo ciclo para criar as referências; a primeira ordem é manutenção das posições lidas. O delta da garra passa a ser calculado entre amostras efetivamente consumidas, mantendo o mesmo fator 0,5 e `GripperVelocityToJoint(speed_factor=20)` que já existem.

### Candidato do produtor de visão

O seguinte substituiria `frame_display()` no mesmo ficheiro; pressupõe a alteração de `frame_capture()` e as funções/variáveis anteriores. Conserva os pontos numerados e o cálculo de garra atual. Passou o ensaio integrado básico descrito abaixo com mão e robô simulados; ainda não foi aplicado ao controlador nem testado com dispositivos reais.

```python
def frame_display():
    filtered_pos = filtered_depth = filtered_grip = None
    filter_epoch = None
    last_video_timestamp = 0
    while not stop_event.is_set():
        try:
            captured_at, frame = frame_queue.get(timeout=0.05)
        except queue.Empty:
            continue

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        timestamp_ms = max(time.monotonic_ns() // 1_000_000, last_video_timestamp + 1)
        last_video_timestamp = timestamp_ms
        results = hands.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=image), timestamp_ms
        )
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        height, width = image.shape[:2]
        landmarks = results.hand_landmarks[0] if results.hand_landmarks else None
        geometry = hand_geometry(landmarks, width, height) if landmarks else None
        valid = geometry is not None and 0 <= time.monotonic() - captured_at <= MAX_FRAME_AGE

        if valid:
            b_angle, roll_angle, center, forward = geometry
            position = to_vec(landmarks[2])
            depth = depth_estimate(landmarks)
            thumb = to_vec(landmarks[4]) - to_vec(landmarks[0])
            finger = to_vec(landmarks[8]) - to_vec(landmarks[0])
            if np.linalg.norm(thumb) == 0 or np.linalg.norm(finger) == 0:
                valid = False
            else:
                grip = angle_between_vectors(thumb, finger)
                valid = bool(np.isfinite(position).all() and
                             np.isfinite([depth, grip, b_angle, roll_angle]).all())

        if valid:
            with tracking_lock:
                if not tracking_state["visible"]:
                    tracking_state["epoch"] += 1
                tracking_state["visible"] = True
                tracking_state["captured_at"] = captured_at
                epoch = tracking_state["epoch"]
            if filter_epoch != epoch:
                filtered_pos, filtered_depth, filtered_grip = position, depth, grip
                filter_epoch = epoch
            else:
                filtered_pos = 0.9 * filtered_pos + 0.1 * position
                filtered_depth = 0.9 * filtered_depth + 0.1 * depth
                filtered_grip = 0.9 * filtered_grip + 0.1 * grip

            for connection in mp.tasks.vision.HandLandmarksConnections.HAND_CONNECTIONS:
                start, end = landmarks[connection.start], landmarks[connection.end]
                cv2.line(image, (int(start.x * width), int(start.y * height)),
                         (int(end.x * width), int(end.y * height)), (0, 255, 0), 2)
            for index, landmark in enumerate(landmarks):
                point = (int(landmark.x * width), int(landmark.y * height))
                cv2.circle(image, point, 3, (0, 255, 0), -1)
                cv2.putText(image, str(index), (point[0] + 4, point[1] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.arrowedLine(image, tuple(center.astype(int)),
                            tuple((center + 50 * forward).astype(int)), (0, 0, 255), 2)
            action = {"epoch": epoch, "captured_at": captured_at,
                      "hand_pos": filtered_pos.copy(), "hand_depth": float(filtered_depth),
                      "b_angle": b_angle, "roll_angle": roll_angle,
                      "gripper_angle": float(filtered_grip)}
            try:
                action_queue.put_nowait(action)
            except queue.Full:
                pass
        else:
            with tracking_lock:
                if tracking_state["visible"]:
                    tracking_state["visible"] = False
                    tracking_state["epoch"] += 1

        with tracking_lock:
            status = tracking_state["status"]
        cv2.putText(image, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0) if status == "ACTIVE" else (0, 0, 255), 2)
        cv2.imshow("SO-101 Hand Tracking", image)
        if cv2.waitKey(5) & 0xFF == 27:
            stop_event.set()
```

A cópia candidata dos processadores no consumidor ainda partilha o solver PlaCo; é necessário conferir o estado desse solver e a ordem da observação. Não aplicar o produtor isoladamente ao controlador atual: o consumidor atual espera outras chaves.

### Candidato do consumidor

O bloco seguinte substituiria `robot_process()` em `Brosnantest/hand_camera_teleop.py`. Depende do produtor e do contrato acima. `trial.steps[1]` é o `EEReferenceAndDelta` na ordem atual da cadeia; não fazer reset da cadeia inteira, que também apagaria a proteção cartesiana existente.

```python
def robot_process():
    pipeline = hand_to_robot_joints_processor
    names = list(robot.bus.motors)
    anchor = None
    held = True
    try:
        while not stop_event.is_set():
            start = time.monotonic()
            with tracking_lock:
                if (tracking_state["visible"] and
                        start - tracking_state["captured_at"] > MAX_FRAME_AGE):
                    tracking_state["visible"] = False
                    tracking_state["epoch"] += 1
                visible = tracking_state["visible"]
                epoch = tracking_state["epoch"]

            if not visible:
                anchor = None
                with tracking_lock:
                    tracking_state["status"] = "PAUSED"
                if not held:
                    reading = robot.get_observation()
                    hold = {f"{name}.pos": reading[f"{name}.pos"] for name in names}
                    with tracking_lock:
                        if not tracking_state["visible"]:
                            robot.send_action(hold)
                            held = True
                            print("PAUSED: mão indisponível", flush=True)
                stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
                continue

            sample = None
            while True:
                try:
                    sample = action_queue.get_nowait()
                except queue.Empty:
                    break
            if (sample is None or sample["epoch"] != epoch or
                    time.monotonic() - sample["captured_at"] > MAX_FRAME_AGE):
                stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
                continue

            reading = robot.get_observation()
            obs = {f"{name}.pos": reading[f"{name}.pos"] for name in names}
            trial = copy.copy(pipeline)
            trial.steps = [copy.copy(step) for step in pipeline.steps]
            fresh = anchor is None or anchor["epoch"] != epoch
            if fresh:
                proposed = {"epoch": epoch, "pos": sample["hand_pos"].copy(),
                            "depth": sample["hand_depth"], "b": sample["b_angle"],
                            "roll": sample["roll_angle"], "grip": sample["gripper_angle"],
                            "m1": obs["shoulder_pan.pos"], "m5": obs["wrist_roll.pos"]}
                trial.steps[1].reset()
            else:
                proposed = anchor.copy()

            targets = (
                proposed["m1"] + angle_delta(sample["b_angle"], proposed["b"]),
                proposed["m5"] + angle_delta(sample["roll_angle"], proposed["roll"]),
            )
            current_fixed = (obs["shoulder_pan.pos"], obs["wrist_roll.pos"])
            if not fixed_targets_in_bounds(current_fixed, fixed_bounds):
                raise RuntimeError(f"leitura M1/M5 fora da calibração: {current_fixed}")
            if not fixed_targets_in_bounds(targets, fixed_bounds):
                raise RuntimeError(f"alvo M1/M5 fora da calibração: {targets}")
            kinematics_solver.fixed_targets = targets
            camera_action = {
                "enabled": True,
                "target_x": sample["hand_pos"][0] - proposed["pos"][0],
                "target_z": -(sample["hand_pos"][1] - proposed["pos"][1]),
                "target_y": -(sample["hand_depth"] - proposed["depth"]),
                "target_wx": 0.0, "target_wy": 0.0, "target_wz": 0.0,
                "gripper_vel": (sample["gripper_angle"] - proposed["grip"]) * 0.5,
            }
            joint_action = trial((camera_action, obs))
            proposed["grip"] = sample["gripper_angle"]
            with tracking_lock:
                valid = (tracking_state["visible"] and tracking_state["epoch"] == epoch and
                         0 <= time.monotonic() - sample["captured_at"] <= MAX_FRAME_AGE)
                if valid and not stop_event.is_set():
                    requested = obs if fresh else joint_action
                    sent = robot.send_action(requested)
                    if sent != requested:
                        raise RuntimeError("o comando enviado difere do calculado")
                    anchor, pipeline, held = proposed, trial, False
                    tracking_state["status"] = "ACTIVE"
                    if fresh:
                        print("ACTIVE: nova referência", flush=True)
            stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
    except Exception as exc:
        print(f"Erro no controlo do braço: {exc}", flush=True)
        stop_event.set()
```

O lock mantém a verificação e a chamada serial na mesma secção; uma escrita já iniciada não pode ser cancelada e pode atrasar a atualização da visão. Uma exceção serial pode ocorrer após uma escrita parcial: o código informa e termina, sem garantir que o braço parou. Ao retomar, a proteção `EEBoundsAndSafety` conserva o histórico; se rejeitar a nova referência, o erro deve ser informado, não contornado.

**Verificações básicas feitas no container temporário:** os blocos passaram análise de sintaxe; com pontos sintéticos, a geometria devolveu ângulos válidos; com o solver/URDF instalados, a classe manteve os dois alvos impostos. Um ensaio encadeou os trechos literais do produtor e consumidor com PlaCo e processadores LeRobot reais, usando imagens, mão, calibração e robô simulados: entrada, movimento, perda e retorno passaram; o estado voltou a `ACTIVE`, um alvo fora da calibração sintética foi rejeitado, e `PAUSED` foi registado na perda. A interface gráfica foi simulada. Nenhum ensaio comprova a resposta da câmara, serial ou braço físico.

No teste sem hardware, verificar perda, amostra expirada, ausência de repetição de M6, retorno noutro ponto e rejeição de uma ação antiga. Simular também falha de leitura/escrita e duas pausas consecutivas. O ensaio integrado básico acima cobre apenas uma perda e um retorno; as restantes situações continuam por verificar antes de as considerar comprovadas.

## Sequência quando a implementação for autorizada

1. Inserir o cálculo de B e da linha 5–17 no ramo de landmarks já existente; desenhar apenas B e conservar os pontos numerados. Fazer a primeira deteção guardar as referências.
2. Enviar os dois ângulos na ação existente para a thread do robô e transformá-los em alvos relativos às leituras de M1 e M5 na referência. Usar as mesmas unidades de graus esperadas pelo LeRobot.
3. Mascarar M1/M5 no solver e adaptar apenas a entrada da IK conforme acima. Preservar o cálculo de M6 e a cadeia, calibração, modelo e hardware existentes.
4. Implementar a pausa e a retomada descritas acima, incluindo referências novas e invalidação das amostras antigas.
5. Verificar no container, sem hardware, a separação de M1/M5, a IK de M2–M4, o comportamento de M6 e os cenários de pausa e retomada. Verificar também um alvo que a IK não consiga atingir e informar o resultado.
6. Depois, com o operador presente, observar o sinal e a sensibilidade da seta e da linha antes de movimentos pequenos; conferir perda e retomada. Ajustar sinal/escala conforme o resultado. Esc encerra normalmente; o corte físico deve estar acessível.

## Fontes e confirmação

- [Implementação de `RobotKinematics` no LeRobot](https://github.com/huggingface/lerobot/blob/v0.7.0/src/lerobot/model/kinematics.py).
- [Documentação PlaCo: mascarar juntas](https://placo.readthedocs.io/en/v0.10.2/kinematics/joints_mask.html).
- [OpenCV: ajuste de linha com `fitLine`](https://docs.opencv.org/2.4.3-rc/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html): o plano usa o método de mínimos quadrados já instalado.
- [LeRobot: teleoperação de telefone](https://github.com/huggingface/lerobot/blob/main/docs/source/phone_teleop.mdx): referência para reancorar a posição, não código acrescentado ao projeto.
- Inspeção do container existente e simulação isolada: com M1 e M5 fixados, M2–M4 ajustaram um alvo vertical no caso testado. Isso não demonstra a resposta da mão real nem que qualquer alvo será alcançável.
