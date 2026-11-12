# TarefasHumanas — calibração e aceitação da teleoperação SO-101

Este ficheiro é o teu plano paralelo ao [plano de implementação](plans/2026-11-10-1200-refactor-calibrated-hand-teleop-plan.md). Os identificadores H01–H08 são iguais nos dois documentos. Cada tarefa diz quando a preciso, o que fazer e o que entregar. Eu preparo os programas, conduzo as capturas e analiso os resultados; não precisas calcular matrizes nem atribuir nomes aos 21 pontos.

## Ordem e trabalho em paralelo

| Tarefa | Podes preparar | Preciso da entrega em | Entrega |
|---|---|---|---|
| H01. Confirmar modo da webcam | Já | Antes da captura U1 e medição U3 | Identificar câmara, resolução e foco/zoom usados; nenhuma medição câmara→robô |
| H02. Imprimir tabuleiro | Já | Antes da captura U1 | Tabuleiro plano e largura real dos quadrados em mm |
| H03. Mostrar tabuleiro à câmara | Após H01–H02 e quando eu abrir o capturador U1 | Conclusão U1 | Pelo menos 15 vistas válidas, guardadas por mim |
| H04. Posições medidas da mão | Preparar régua, apoio e folha com marcas já; capturar quando o módulo de U1 estiver pronto | Antes de concluir U1 e iniciar U3 | Profundidades e deslocamentos laterais/verticais medidos, com amostras etiquetadas |
| H05. Conferir convenção de movimento | Após U3 | Antes da integração cartesiana U5 | Mão direita/cima/perto da webcam medida com sinais corretos; nenhum alinhamento espacial com o robô |
| H06. Localizar centro físico da garra | Fotos e régua podem ser preparados já; captura/medida conjunta em U4 | Antes da integração cartesiana U5 | Medidas e fotos da garra em dois estados |
| H07. Aceitar XYZ no braço | Apenas quando U5/U6 estiver pronto | Gate físico U6 | Estar ao lado do braço e relatar movimento real por eixo |
| H08. Aceitar orientação da garra | Apenas depois de H07 passar | Gate físico U7 | Estar ao lado do braço e relatar direção real da pinça |

H01, H02 e a preparação de régua/apoios para H04 e H06 podem ocorrer enquanto eu refatoro o código. H03–H08 são sessões conjuntas: combinamos a etapa quando o software de captura ou o modo live correspondente estiver pronto. A câmara não precisa ver o SO-101 nem ter posição fixa relativamente a ele. Nunca inicio um movimento físico só porque uma tarefa está escrita neste plano.

## H01. Confirmar o modo da webcam

**Quando:** antes de H03 e H04; repetir só se a configuração óptica ou o dispositivo mudar.

**Faz:** confirma qual webcam será usada (`/dev/video0` é a candidata atual), a resolução efetiva, o zoom e se há autofoco. Mantém o mesmo modo óptico durante a calibração intrínseca e as medições da mão; deixa espaço para a palma e para o tabuleiro aparecerem inteiros. Não precisas posicionar ou marcar o portátil em relação à mesa ou ao SO-101. Podes mover o portátil e o robô entre sessões; cada PALM aceite cria uma nova origem relativa. Durante um gesto ativo, mantém a webcam parada ou pede HOLD e ativa outra PALM depois de a moveres. A base do robô só precisa ficar mecanicamente estável enquanto ele se move. Não mexas nos cabos dos servos com USB ou fonte ligados.

**Entrega:** diz-me «H01 pronto: webcam=…; resolução=…; foco/zoom=…». Eu verifico a câmara real e anoto o perfil. Mudar apenas a posição do portátil ou da base não exige recalibrar a webcam; trocar dispositivo, foco, zoom ou resolução pode exigir nova calibração intrínseca e de escala da mão.

## H02. Imprimir e medir o tabuleiro

**Quando:** podes fazê-lo já; é necessário para H03/U1.

**Ficheiro:** [tabuleiro A4 pronto para impressão](calibration-board-9x7-18mm.svg). É um xadrez com **10 colunas × 8 linhas de quadrados**, logo **9 × 7 cantos internos**. Cada quadrado nominal mede **18 × 18 mm**; a malha inteira mede **180 × 144 mm** e cabe numa folha A4 com margem branca. O pequeno ponto fora da malha marca o canto de referência; não é um canto interno.

**Faz:** imprime A4 a **100% / tamanho real**, sem «ajustar à página», sem escala automática e sem cortar a malha. Mede com régua a linha de referência de 90 mm e pelo menos três quadrados em sítios diferentes. Anota largura medida, por exemplo `17,9; 18,0; 18,0 mm`, e verifica que a malha não está esticada de maneira diferente na horizontal e vertical. Cola a folha sem vincos numa placa/cartão rígido e plano, de superfície mate. Mantém os 9 × 7 cantos internos visíveis.

**Entrega:** «H02 pronto: linha=… mm; quadrados horizontais=… mm; verticais=… mm». Se alguma medida diferir mais que ~1% da nominal, avisa antes de H03: posso usar as dimensões reais se forem uniformes, ou pedir nova impressão se estiver distorcida. Não uses o número 18 mm no cálculo sem a medida do papel. [OpenCV descreve cantos internos, padrão plano e medição do lado real](https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html).

## H03. Capturar intrínsecos da câmara

**Quando:** depois de H01 e H02, quando eu disser «U1 pronto para H03» e abrir a captura no container.

**Faz durante a sessão:** segura o tabuleiro sem tapar a malha. Começa de frente, depois inclina e roda o tabuleiro; move-o para centro, quatro regiões laterais/cantos e diferentes distâncias em que **todos os cantos internos continuem nítidos**. Mantém-no imóvel um instante em cada pose para eu aceitar a imagem. Procuraremos 20 vistas válidas: 15 para ajuste e 5 reservadas para verificação. Se só conseguirmos 15, usaremos 12 para ajuste e 3 reservadas; OpenCV recomenda pelo menos dez boas vistas em posições diferentes. Mantém foco/zoom/resolução no modo H01; a webcam pode estar em qualquer lugar e o robô pode estar fora do quadro. A distância desta tarefa é escolhida pela nitidez do tabuleiro, não precisa coincidir com os 30–60 cm da mão.

**Entrega:** eu guardo imagens e resultados sob `data/`; tu só precisas manter o tabuleiro e responder se a posição continua estável. Concluímos H03 quando o detector encontrar os 9 × 7 cantos, houver cobertura da imagem, erro de reprojeção documentado e imagens de reserva não usadas no ajuste. Meta inicial de qualidade: RMS de reprojeção do ajuste ≤1 pixel e erro p95 nas vistas reservadas ≤2 pixels; se falhar, repetimos capturas ou revemos rigidez/iluminação antes de aceitar a calibração.

**Capturador preparado:** `scripts/calibrate_hand_camera.py` no container abre só a webcam. Depois de me dizeres o lado real do quadrado em mm e o modo de foco/zoom H01, eu inicio a captura; pressionamos `f` para vistas de ajuste, `h` para vistas reservadas e `q` para terminar. Cada imagem original e cantos ficam em `data/hand_camera/`. O ajuste é um comando separado e não altera a calibração dos motores.

## H04. Fornecer profundidade independente da mão

**Quando:** prepara uma régua/trena, um apoio e uma folha rígida com marcas agora; fazemos a captura quando o caminho de gravação de U1 estiver pronto. H04 fecha o conjunto de amostras de U1 antes de U3 escolher a fórmula de profundidade.

**Faz durante a sessão:** usa sempre a mesma mão. Coloca o **centro da palma**, não a ponta dos dedos, aproximadamente nos planos de **30, 40, 50 e 60 cm** da frente da lente da webcam; se algum plano não couber no campo de visão, registamos o intervalo real utilizável. Usa uma régua perpendicular à câmara ou um apoio/placa com marca de distância e alinha nela o plano aproximado do centro da palma, sem a tapar. Faz três permanências imóveis de cerca de 30 frames em cada distância, pelo menos com a palma frontal; repete alguns pontos com a palma ligeiramente rodada (~15°) e em zonas esquerda/direita da imagem. Reserva comigo duas distâncias intermédias, por exemplo 35 e 55 cm, **sem as usar no ajuste**, para avaliação; nessas posições também registamos sequências estacionárias. Eu marco a distância medida, o desvio entre a marca e o plano real da palma e a incerteza de colocação. Quando a mão rodar, a referência física do centro pode mudar de plano: registamos isso e podemos restringir a calibração métrica às vistas frontais. A frente da lente não coincide necessariamente com o centro óptico; ajustarei esse desvio constante e avaliarei a estimativa nas distâncias reservadas.

**Para medir X/Y independentemente:** numa folha/cartão rígido atrás da mão, traça com a régua uma cruz e marcas a 5, 10 e 20 cm para a esquerda, direita, cima e baixo. Mantém a folha aproximadamente paralela ao plano da webcam, em duas das distâncias utilizáveis. Alinha visualmente o centro da palma com cada marca, mantendo a palma perto do mesmo plano; deixa a mão visível para o MediaPipe. Para Z, aproxima e afasta a palma depois da ativação e observamos a **razão de tamanho aparente** em que o gesto ainda é confortável, visível e estável; não precisamos medir 20 cm físicos de profundidade para definir o comando Z. Eu registo as distâncias à lente para avaliar a projeção X/Y e verificar se a escala Z é monotónica, a inclinação da folha, a distância entre palma e folha e a incerteza do alinhamento. Repetimos algumas séries completas em outras marcas **sem as usar no ajuste**. Se a webcam não observar os ±20 cm completos em X/Y, anotamos o intervalo utilizável e não afirmamos cobertura total. Não fingimos conhecer a posição absoluta da palma no mundo.

**Além disso:** mede com régua a largura visível da tua palma junto à base dos dedos e passa-me o número em mm como controlo aproximado. A distância exata entre os landmarks 5 e 17 não é diretamente mensurável na pele; o software ajustará uma escala efetiva com as distâncias independentes e validará em capturas reservadas. Mostra também punho, mão de lado e «jacaré» para eu medir quando a geometria deixa de ser fiável; essas poses não são, por si, medidas métricas de profundidade.

**Entrega:** «H04 pronto» após eu confirmar que as amostras têm distância à lente, deslocamentos X/Y medidos pelas marcas, orientação, repetição e incerteza anotadas. Se a localização física da palma for imprecisa, registamos a incerteza e não afirmamos precisão superior a ela. Os valores de 30–60 cm são proposta de trabalho, sujeitos ao campo de visão real.

**Capturador preparado:** cada permanência usa uma cópia preenchida de `teleop/hand_sample_label.example.json`. A distância e qualquer deslocamento são números em **metros**; `measured_dx_m` é positivo para a direita e `measured_dy_m` positivo para baixo no vídeo **sem espelho**. Uma permanência de referência tem `dx=0, dy=0`; as deslocadas usam o mesmo `session_id`, e a sessão inteira fica em `fit` ou `held_out`. Marcar `use_for_metric=true` nas poses usadas para medir profundidade, cada uma presente em ambos os conjuntos e com pelo menos um deslocamento XY medido e diferente de zero no conjunto reservado. Marcar `false` nas poses negativas, como punho e mão quase de lado. Ao pressionar `s` no preview, guardamos 30 frames válidos em `data/hand_samples/`. Os campos desconhecidos permanecem `null` até serem medidos; o programa rejeita uma profundidade ausente. Confirmar manualmente que o foco/zoom e a mão continuam os mesmos ao usar o perfil.

## H05. Conferir os sentidos dos comandos

**Quando:** depois de U3 estimar XYZ da mão e antes de U5 usar esses XYZ para mandar o robô mover. Não é uma calibração da posição da webcam em relação ao SO-101.

**Faz durante a sessão:** com o braço **sem receber comandos**, mostra a palma e desloca-a para a direita na imagem espelhada, depois para cima e, por fim, em direção à lente, um sentido de cada vez. Compara marcas medidas de 10 e 20 cm desde a PALM; eu mostro no terminal o deslocamento medido e a fração do curso de mão `[-20,+20] cm`. Explicito a convenção que fará a garra ir para a direita, subir e avançar relativamente à base do robô. O espelho da janela altera a apresentação, por isso o sinal fica definido no mapeamento de controlo e é conferido depois no teste físico H07. Podes fazer esta etapa com a webcam e o robô em quaisquer posições independentes.

**Entrega:** três deslocamentos vistos pela webcam e confirmação da direção desejada da garra. Não preciso medir uma rotação ou translação câmara→base. Se a câmera estiver rodada, colocamos a imagem na orientação correta ou explicitamos uma nova convenção de eixos antes de aceitar o live.

## H06. Medir o centro onde a garra agarra

**Quando:** podes preparar agora; fazemos a medição conjunta em U4 antes de U5.

**Faz:** com o braço sem movimento e apoiado, identifica comigo o ponto médio entre as duas mandíbulas na região de contacto do objeto. Antes de medir, envia uma foto próxima da garra: identificarei no CAD um ponto **visível e fixo**, preferencialmente o centro circular do eixo `wrist_roll`/Motor 5, que o URDF liga ao `gripper_link`; se esse ponto não for identificável, escolhemos outro furo/parafuso com correspondência comprovada no CAD. A partir dessa referência, mede com régua ou paquímetro os deslocamentos nas três direções locais da garra até ao ponto de contacto, ou fornece fotos ortogonais com escala para eu extrair essas medidas. Repete em duas aberturas e em mais de uma pose estática segura. Fotografias sem referência CAD e sem escala não bastam para definir `T_G_T`. Não forces as mandíbulas nem mudes cabos alimentados.

**Entrega:** medidas/fotos e a identificação do ponto de contacto. O offset relativo à garra será confrontado com o CAD; para testar a exatidão absoluta do FK, faremos ainda observações do centro da garra medidas relativamente à **base fixa** do SO-101 em poses estáticas, com régua/fotos de escala. Se a posição do centro variar muito com a abertura ou a referência CAD não for inequívoca, o modelo de offset fixo não pode ser aceite. Uma foto com caminho absoluto serve para eu analisar, mas não substitui a medida de escala.

## H07. Aceitação física de XYZ

**Quando:** só quando U5/U6 estiver pronto, H01–H06 concluídas e eu anunciar o comando live específico. Não precisas restaurar as posições físicas usadas nas calibrações; a PALM ancora a posição atual.

**Faz:** fica ao lado do SO-101, apoia-o, retira carga, liberta o trajeto e deixa o corte de energia acessível. Antes de mover, coloca fora do trajeto uma régua ou papel milimétrico fixo paralelo ao eixo testado; marca/fotografa o centro da garra no início e no fim a partir do mesmo ponto de observação, com escala visível. Para a altura usa régua vertical; para profundidade, régua paralela ao eixo correspondente. Mostra PALM para ancorar. Começa com um movimento pequeno por vez: direita/esquerda na imagem, cima/baixo, aproximar/afastar da webcam; só depois percorre frações maiores do curso de mão de até 20 cm por sentido e testa uma combinação. A garra deve seguir os eixos de comando definidos em H05 e aproximar-se do limite **alcançável** na direção pedida quando a mão chega a 20 cm, independentemente de onde a webcam está colocada. Diz-me o **movimento real do centro da garra** e a incerteza aproximada da leitura, não apenas o que aparece no ecrã. Eu comparo com o alvo, o FK do readback e a distância física; paro no primeiro sentido ou magnitude inesperado. FIST/SPACE pedem HOLD, mas não substituem corte elétrico.

**Entrega:** para cada eixo, «mão +… cm → garra +… cm / direção…» e qualquer vibração/deriva. Se não mover ou houver erro de limite, esta etapa falha e fica em preview até corrigirmos.

## H08. Aceitação da direção da pinça

**Quando:** somente após H07 passar e U7 implementar orientação no mesmo alvo de IK.

**Faz:** repete as condições físicas de H07. Primeiro mantém o centro da palma quase parado e inclina a mão em pequenas quantidades, uma rotação de cada vez. Observa para onde a pinça aponta e se o centro físico da garra permanece na posição aceite. Eu comparo normal/eixo da mão, orientação alvo, juntas e centro por FK; poses fora do alcance devem ser rejeitadas no ecrã.

**Entrega:** confirmação de direção e deriva física aproximada; se a garra deslocar o centro indevidamente ou tentar pose impossível, desligamos a orientação e mantemos o XYZ aceite.

## Fontes e limite das medições

- [MediaPipe Hand Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker?tab=Python): 21 pontos na imagem e pontos 3D relativos ao centro da mão; não fornece a posição absoluta da mão em relação à webcam.
- [OpenCV: padrão de calibração](https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html) e [calibração da câmara](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html): cantos internos, dimensões reais, várias vistas e erro de reprojeção.
- [OpenCV: modelo de projeção e PnP](https://docs.opencv.org/4.10.0/d9/d0c/group__calib3d.html): a escala 3D depende de geometria física conhecida. Área aparente varia com orientação/oclusão; será comparada com span estável e avaliada em dados reservados, não adotada por intuição.
