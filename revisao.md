# Revisão: perguntas e ideias sobre a teleoperação

## 9. Proposta de garra e pausa automatica — 22 de novembro

**Espaço fixo e seta entre dedos:** o operador quer ler juntas e calcular a posição estimada da garra no início, mantendo fixo o referencial 3D da base. Propõe uma seta na região entre polegar e médio que continue a indicar frente ao abrir e fechar. Uma bissetriz calculada com as pontas pode girar quando só um dedo mexe; uma seta baseada nos pontos da palma pode ser mais estável, mas não é garantida e pode apontar numa direção visual diferente. A posição gráfica da seta e a direção usada no controlo podem ser calculadas separadamente. As duas hipóteses e o mapeamento incremental estão no [plano](docs/plano-teleoperacao-pausa.md); falta observá-las e decidir.

**Direção variável:** o operador quer que a direção para onde os dedos apontam seja acompanhada continuamente, de forma que, se apontarem à esquerda da imagem, um deslocamento à esquerda some avanço lógico e um deslocamento à direita subtraia. A direção vertical permanece separada. O [plano](docs/plano-teleoperacao-pausa.md) propõe verificar um vetor da palma, evitar inversões abruptas do alvo, pausar quando a direção for ambígua e definir a correspondência com os eixos da base do robô. Com RGB 2D não se conhece automaticamente a direção física 3D. Só a numeração visual dos pontos foi implementada nesta etapa.

**Hipótese do operador:** apresentar a mão de lado e a pinça fechada no início, como uma garra. Pode ser uma pose confortável e reprodutível se os pontos forem detetados; falta comparar a sua visibilidade com outras orientações. O código não reconhece uma classe de pinça nem associa o ângulo inicial à abertura absoluta da garra: regista-o como referência para diferenças posteriores. Uma pinça inicialmente fechada não garante que a garra física feche. A posição do polegar também influencia translação, escala aparente e orientação. Uma visão perfeitamente lateral pode ocultar pontos; testar na janela antes de concluir que é mais fiável.

O ângulo atual usa 0→4 e 0→8 e a sua **variação** comanda a garra incrementalmente. A proposta usa 9→4 e 9→12 e poderia mapear o ângulo **absoluto** para a abertura. Palma, jacaré e pinça não são classificadas pelo código atual; qualquer mão detetada inicia a referência. Para comparar gestos, observar pontos visíveis e reconhecer oclusão e perspetiva.

O operador prefere que, depois de a mão voltar ao quadro, o sistema retome automaticamente após estabilidade. "Estável" é uma hipótese parametrizável de pouca variação do ponto escolhido durante uma janela, por exemplo 0,5 s e 2% da diagonal; os valores ainda não foram medidos. A referência da mão e da pose do robô precisa de ser renovada; o último delta da garra não pode ser repetido. Ver [plano proposto](docs/plano-teleoperacao-pausa.md). Nenhuma destas mudanças foi implementada.

Registo para estudo e eventual atualização do PDF. As sugestões estão **por decidir**; nenhuma mudança no controlo foi implementada nesta revisão. Base: código local inspecionado em 18 de novembro de 2026; o comportamento físico precisa de verificação separada.

## 1. Por que duas filas? O que o MediaPipe devolve?

**Pergunta.** Por que há duas filas e quem calcula as posições, escalas e ângulos? É uma chamada de API?

**Resposta.** Uma fila transporta imagens da thread de captura para a de visão; outra transporta ações calculadas pela visão para a thread do robô. Isso permite que as tarefas avancem em ritmos diferentes, mas acumular dados pode aumentar o atraso. São filas locais em memória, não pedidos pela internet. A primeira aceita até 5 frames e a segunda até 100 ações. Quando estão cheias, o código deixa de acrescentar o novo item, preservando os antigos.

O OpenCV lê a imagem; o programa converte BGR para RGB e chama localmente a API Python `hands.detect_for_video(...)`. O MediaPipe executa os modelos no computador e devolve um objeto `results`; `results.hand_landmarks` contém pontos da mão com coordenadas acessíveis como `landmark.x`, `.y` e `.z`. O código atual usa apenas `.x` e `.y` para a sua matemática. O próprio código calcula posição relativa, somas de distâncias, ângulos e suavização; eles não chegam prontos como comandos do MediaPipe. O timestamp usado é feito no processamento, não no momento da captura.

Fontes: [captura e filas](Brosnantest/hand_camera_teleop.py#L119), [chamada do MediaPipe](Brosnantest/hand_camera_teleop.py#L194), [geometria e ações](Brosnantest/hand_camera_teleop.py#L231).

**Para decidir depois:** avaliar substituir as filas por dados mais recentes e adicionar idade de captura; não assumir que isso, sozinho, elimina atraso do driver ou de inferência.

## 2. O que é uma thread? Como se lê a posição dos motores?

**Pergunta.** O que é a thread do robô, como sabe a posição e que espaço virtual usa?

**Resposta.** Uma thread é um percurso de execução dentro do mesmo programa. Aqui há três: captura da câmara, processamento e visualização, e comunicação/controlo do robô. Elas partilham dados via filas e podem avançar em ritmos diferentes. Não são três programas independentes nem três chamadas ao MediaPipe.

A thread do robô usa `robot.get_observation()` do LeRobot. Pela ligação USB/serial, o LeRobot comunica com os servos Feetech e lê as suas posições reportadas; a calibração converte os valores para a convenção da aplicação. Não é o MediaPipe que lê os motores. São posições articulares relatadas pelos servos, não posições XYZ de cada motor medidas por uma câmara externa.

O URDF é um modelo geométrico do braço, com ligações, juntas, transformações e limites. A cinemática direta combina o modelo e os ângulos lidos para estimar a pose de um referencial escolhido da garra relativamente à base do modelo. Portanto existe um espaço 3D matemático, cuja fidelidade depende do modelo, montagem e calibração; não é uma observação externa perfeita da ponta nem inclui automaticamente a mesa e obstáculos.

Fontes: [configuração da cinemática](Brosnantest/hand_camera_teleop.py#L42), [ciclo do robô](Brosnantest/hand_camera_teleop.py#L284), [modelo URDF](teleop/models/so101/so101_new_calib.urdf).

## 3. Quem calcula quanto cada motor se move?

**Pergunta.** Como se converte a nova posição desejada numa ação para cada motor?

**Resposta.** O código transforma o movimento estimado da mão num alvo para o referencial da garra. A cadeia LeRobot usa a pose de referência e os limites configurados; `RobotKinematics`/PlaCo resolve a cinemática inversa: procura valores das juntas coerentes com o alvo e o modelo. O LeRobot envia esses objetivos de posição aos servos. Cada servo, por sua vez, executa internamente controlo de posição a partir do objetivo e da sua medição. O código local não mede força nem prova que o destino foi atingido quando a escrita é aceite. Uma solução IK pode ter erro, encontrar outras configurações ou não ser alcançável; não se deve confundir o alvo calculado com uma medição externa de onde a garra está.

Fontes: [cadeia de processadores](Brosnantest/hand_camera_teleop.py#L57), [envio da ação](Brosnantest/hand_camera_teleop.py#L312).

## 4. Quem cria o XYZ e o que é o ponto 2?

**Pergunta.** Os landmarks vêm de quem, podemos mudar a matemática, e qual ponto do robô o ponto 2 controla?

**Resposta.** Para cada imagem processada, os modelos do MediaPipe estimam os landmarks da mão. Depois o nosso Python escolhe o ponto 2, na base do polegar, filtra `x` e `y`, subtrai os valores da primeira deteção e calcula uma variação de escala para o terceiro comando. Podemos investigar outras combinações dos pontos sem alterar o MediaPipe; p. ex., o centro de pontos da palma. O sinal do ponto 2 é um *deslocamento relativo de entrada*, não uma correspondência física 1:1 direta com uma ponta do robô.

A IK visa `gripper_frame_link`, referencial definido no URDF e ligado por junta fixa a `gripper_link`. É uma posição definida pelo modelo para a ferramenta. Não afirmamos que coincide exatamente com o centro do motor 6, com a ponta física ou com o centro variável entre as duas pontas: isso exige analisar o modelo e medir o braço montado.

Fontes: [seleção do referencial](Brosnantest/hand_camera_teleop.py#L50), [ponto 2](Brosnantest/hand_camera_teleop.py#L240), [referencial no URDF](teleop/models/so101/so101_new_calib.urdf#L293).

**Para decidir depois:** comparar ponto 2, um ponto da palma e centro de vários pontos da palma; média dos 21 pontos também pode mudar quando os dedos se fecham.

## 5. Que movimentos da mão correspondem aos eixos? Como começar?

**Pergunta.** O que é crescer para a direita e qual a posição correta da mão?

**Resposta.** `x` da imagem cresce da esquerda para a direita e `y` de cima para baixo no frame recebido pelo programa. São coordenadas da imagem, não automaticamente as direções físicas do operador; a apresentação pode parecer espelhada ou não, dependendo da câmara e da forma como se olha. O código envia `Δx` da imagem ao alvo X do robô, `-Δy` da imagem ao alvo Z e `-Δescala` ao alvo Y. A orientação da câmara e da base determina como isso se traduz em esquerda/direita no mundo. Também há uma componente de orientação derivada do polegar. As respostas físicas e sinais dependem da montagem e ainda não foram validados em todos os eixos.

O programa atual não pede nem valida uma pose inicial específica. Usa a primeira mão detetada, sem distinguir palma, punho ou jacaré. Manter mão visível, numa pose confortável e com espaço para mover é uma orientação prática provisória, não uma pose garantida como correta. O primeiro teste posterior deve verificar um eixo de cada vez, com supervisão e corte de energia acessível; não fazer o teste automaticamente nesta revisão.

Fonte: [mapeamento dos eixos](Brosnantest/hand_camera_teleop.py#L303).

## 6. Referências capturadas separadamente e reentrada da mão

**Pergunta.** Qual a diferença prática de ancorar simultaneamente? O que acontece ao sair à direita e voltar pela esquerda?

**Resposta.** A referência do braço é definida pela cadeia LeRobot quando o loop com `enabled=True` começa; a referência da mão é guardada quando a primeira mão é detetada. Não são registadas por uma única ação de ativação. Assim, o estado inicial do braço e a primeira posição da mão podem pertencer a momentos diferentes.

Depois de a mão sair, o código atual não apaga essas referências nem rearma por nova deteção. Ao voltar pelo lado esquerdo, o sistema compara a nova posição com a **primeira posição original**, não com o ponto de reentrada; pode haver salto de alvo. Durante a ausência, conserva a última ação e pode continuar a reaplicar o delta da garra. Isto não é uma pausa segura nem permite, por si só, fazer o gesto de reposicionamento pedido.

**Ideia para decisão:** uma pausa/embreagem (*clutch*) explícita: parar comandos apropriados, mover a mão sem seguir, e ao reativar guardar juntas recentes/pose atual da garra e nova pose da mão juntas. O primeiro comando relativo deve começar em zero. Ainda é preciso definir comportamento em erro de serial: não se pode garantir que um comando de pausa chega ao motor quando a ligação falha.

Fontes: [referência da mão](Brosnantest/hand_camera_teleop.py#L237), [estado inicial e reutilização da ação](Brosnantest/hand_camera_teleop.py#L284).

## 7. Quem calcula `depth_estimate`? Pontos mais estáveis ajudam?

**Pergunta.** A escala é calculada pelo MediaPipe? A abertura da mão não altera o resultado? Podíamos usar pontos que mantenham distância?

**Resposta.** O nosso Python calcula a soma de oito comprimentos 2D entre landmarks devolvidos pelo MediaPipe. O modelo não devolve essa soma. Abrir, fechar, rodar ou ocluir a mão pode mudar o valor sem mudança de distância, portanto a dúvida é fundamentada. Comparar pontos da estrutura da palma, como 5 e 17, pode reduzir interferência dos dedos, mas a distância projetada também varia quando a palma roda e pode sofrer oclusão; não resolve a ambiguidade de profundidade monocular. Com a D435i, profundidade medida e alinhada ao RGB é outra hipótese a avaliar, com rejeição de medições inválidas.

Fonte: [`depth_estimate`](Brosnantest/hand_camera_teleop.py#L165).

## 8. Como funciona hoje a garra? Distância ou ângulo seria mais simples?

**Pergunta.** Como a garra é calculada e poderíamos usar distância ou produto interno?

**Resposta.** Hoje o programa desenha dois vetores 2D a partir do pulso, `0→4` e `0→8`, e calcula o ângulo entre eles pelo produto interno: `cos θ = (a·b)/(|a||b|)`, depois usa `arccos`. Filtra esse ângulo e envia a diferença entre amostras. O ciclo do robô multiplica a diferença por 0,5; o processador LeRobot usa um fator 20 e soma o resultado à posição de garra lida, limitando o alvo a 0–100 na representação desse processador. Logo é um comando incremental, não um mapeamento direto entre abertura atual da mão e abertura atual da garra. O nome `gripper_vel` não demonstra unidades físicas por segundo. Se uma ação não nula for repetida, pode continuar a deslocar o alvo; efeito físico não foi medido aqui.

**Ideia para decisão:** medir a distância entre pontas ou um ângulo de dois vetores e mapear uma faixa de mão aberta/fechada para a faixa de posição calibrada da garra. A distância entre 4 e 12 é um lado de um triângulo, só uma hipotenusa se o triângulo for retângulo. Para o ângulo 4–9–12, os vetores partem do ponto 9 para 4 e para 12; o produto interno dá o cosseno do ângulo. Posições 2D dependem de perspetiva, escala e oclusão. Isto controla um alvo de posição; não comprova força de aperto.

Fontes: [cálculo dos vetores](Brosnantest/hand_camera_teleop.py#L249), [remapeamento](Brosnantest/hand_camera_teleop.py#L303), [processador LeRobot v0.7.0](https://github.com/huggingface/lerobot/blob/v0.7.0/src/lerobot/robots/so_follower/robot_kinematic_processor.py).
