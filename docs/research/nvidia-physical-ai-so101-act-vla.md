# Curso NVIDIA SO-101: análise, câmeras, teleoperação e modelos

**Revisado em 6 de novembro de 2026.** Percorri a trilha principal na ordem das páginas 01–16 e as três páginas de referência. Abri a documentação na aba lateral, conferi visualmente a primeira janela de cada página e inspecionei as imagens, GIFs e players que a árvore da página expõe. Também consultei a playlist ligada pelo índice e suas legendas automáticas em inglês; não reproduzi a transmissão de 1h13 inteira em tempo real.

## Resposta curta à dúvida da câmera

A câmera na garra **não grava o movimento do braço**. Ela grava imagens da cena. O movimento é registrado por outra fonte: o teleoperador gera comandos e o SO-101 registra estado e ações das juntas. O dataset LeRobot sincroniza vídeo, posição articular e ação em cada instante.

São quatro coisas diferentes:

| Sinal ou cálculo | O que fornece |
|---|---|
| Câmera da garra | Imagem próxima do objeto e da garra; não fornece, sozinha, a pose global do braço nem escala métrica confiável. Pode ficar tapada pelo objeto. |
| Câmera fixa da cena | Visão contínua de bancada, objeto e destino; reduz a oclusão da câmera da garra. |
| Posição dos motores/juntas | Propriocepção: posição atual de cada junta; é parte do que o modelo observa e do que a demonstração grava. |
| Geometria e cinemática do braço | Um modelo URDF e uma base calibrada permitem calcular a pose da ferramenta por cinemática direta (FK) ou converter uma pose desejada em ângulos por cinemática inversa (IK). |

O curso da NVIDIA não faz reconstrução geométrica completa da bancada para cada ação. Ele treina uma política de visão com imagens, feedback articular e linguagem. Na trilha N1.6 descrita, as saídas são posições articulares em blocos temporais. A rede aprende uma relação entre observação e demonstração; isso não é o mesmo que calcular explicitamente o volume de trabalho ou planejar colisões.

## Como isso se aplica ao nosso braço

O projeto está decidido como **um follower SO-101, sem leader**. Por isso, o fluxo literal do curso não encaixa: a trilha usa um leader físico para teleoperar o follower e, para a tarefa de bancada, duas câmeras. Isso não impede treinar com o nosso braço, mas precisamos escolher outra forma de gerar demonstrações.

Uma alternativa documentada pelo LeRobot é teleoperar com um telefone. O telefone fornece uma pose desejada da ferramenta; a cadeia usa limites de área e velocidade e IK, apoiada no URDF do SO-101, para transformar essa pose em comandos de juntas. Isso corresponde à intuição de controlar posições em x/y/z com matemática. Exige calibrar o referencial telefone–base e validar limites, alcance e movimentos seguros; não vem automaticamente do vídeo da câmera. A documentação oficial inclui exemplos de telefone para SO-100/SO-101.

Também podemos comandar diretamente as juntas. É menos intuitivo para o operador, mas evita IK e ainda produz ações válidas para treinar. Em ambos os caminhos, são os comandos do teleoperador e o estado dos motores que registram o movimento; a câmera registra o que o braço estava vendo.

### ACT ou VLA?

| Abordagem | O que aprende/recebe | Ajuste ao projeto |
|---|---|---|
| Controle geométrico com IK | Um operador manda a ferramenta para poses cartesianas; URDF, FK/IK e limites convertem isso em juntas. | É o caminho para gerar demonstrações sem comprar um segundo braço. Não é um modelo que entende tarefas sozinho. |
| ACT | Imagens e posições articulares entram; o modelo imita demonstrações e prevê um bloco de ações futuras. Não recebe instrução de linguagem como parte normal do ACT. | Melhor primeiro modelo aprendido: LeRobot o recomenda para começar, é mais leve e aceita uma ou mais câmeras. A imagem da garra pode ser suficiente para tarefas simples, mas a oclusão continua sendo uma limitação visual. |
| VLA, como GR00T | Imagens, instrução de linguagem e estado entram; a política produz ações. O pré-treino pode ajudar com conceitos visuais/linguísticos, mas adaptação ao braço, dados, calibração e segurança continuam necessários. | Faz sentido se quisermos variar instruções ou tarefas. É mais complexo e pesado; não resolve por si só a falta de teleoperação, mapa métrico, câmera global ou IK. |

**Recomendação para começar:** primeiro validar comando manual seguro e gravação de dados; depois treinar ACT como referência. Só avançar para VLA quando a tarefa e os dados funcionarem e houver um motivo claro para a linguagem/generalização adicional. A teleoperação cartesiana e o ACT são complementares: IK ajuda a gerar ações matematicamente; ACT aprende a imitar essas ações a partir de imagens/estado.

## O que a trilha faz, na ordem

| Página | Conteúdo e leitura para o nosso caso |
|---|---|
| [01 — Overview](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/01-overview.html) | Define Physical AI e a tarefa de pegar frascos e colocá-los num rack. O modelo usa visão 2D; a página já reconhece que a câmera da garra pode ficar ocluída após pegar o frasco. |
| [02 — How to Take This Course](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/02-how-to-take-this-course.html) | Propõe seguir tudo com modelos/dados prontos, gravar os próprios dados, trocar de tarefa ou remover o lightbox. Para nós, a opção “as-is” depende de workspace e entradas compatíveis. |
| [03 — What Is Sim-to-Real?](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/03-sim-to-real.html) | Separa diferenças de sensoriamento, atuação, física e modelo. Explica por que sucesso no simulador não prova sucesso no braço real. |
| [04 — LeRobot](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/04-lerobot.html) | Explica o SO-101 de seis juntas, o formato de dataset e a convenção leader/follower. O curso espera um leader para teleoperação; nosso escopo não o inclui. |
| [05 — Building the Workspace](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/05-building-workspace.html) | Monta lightbox, iluminação, rack e frascos. Especifica uma câmera na garra e uma câmera externa fixa; o layout e a iluminação são mantidos próximos da simulação. |
| [06 — Get the Code and Models](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/06-get-the-code.html) | Clona o workshop e prepara dois containers. A página fixa Isaac Sim 5.1, Isaac Lab 2.3, LeRobot 0.4.3 e GR00T N1.6. |
| [07 — Calibrating the SO-101](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/07-calibrating-so101.html) | Encontra portas USB, calibra IDs e percorre limites mecânicos. Distingue a fonte de 5 V do leader da de 12 V do follower. Nosso follower pode ser calibrado sozinho, mas a energia tem de ser correta. |
| [08 — Operating the SO-101](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/08-operating-so101.html) | Teleopera com o leader, verifica as duas câmeras e usa Rerun para acompanhar vídeo e estado. Orienta o operador a olhar pelas câmeras que a política verá, não diretamente para o robô. |
| [09 — Strategy 1: Domain Randomization](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/09-strategy1-dr-teleop.html) | Usa o leader para controlar o SO-101 simulado e grava demonstrações; randomiza luz, câmera e objetos. O exemplo usa cerca de 75 demonstrações e recomenda movimentos suaves e dados diversos. |
| [10 — Isaac GR00T: VLA](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/10-groot.html) | Explica VLA e pós-treino de GR00T N1.6. O exercício usa câmeras, estado, instrução e blocos de ações; indica GPU e tempo de treino muito superiores ao baseline ACT. |
| [11 — Sim Evaluation](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/11-sim-evaluation.html) | Avalia o checkpoint no simulador antes do hardware. A própria página reporta aproximadamente 50–70% de sucesso para o baseline de 75 demonstrações simuladas. |
| [12 — Real Evaluation](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/12-real-evaluation.html) | Executa o mesmo modelo no braço, com duas câmeras e controle de parada. Espera erros de percepção e atuação; requer espaço livre e supervisão. |
| [13 — Strategy 2: Co-Training](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/13-strategy2-cotraining.html) | Mistura simulação e demonstrações reais, depois compara uma política ajustada no hardware. Continua exigindo leader, duas câmeras e a infraestrutura GR00T. |
| [14 — Strategy 3: Cosmos](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/14-strategy3-cosmos.html) | Aumenta visualmente vídeos de demonstração para variar aparência/luz. Mantém a trajetória/ações e muda a sequência visual; o curso fornece dados já aumentados. |
| [15 — Strategy 4: SAGE + GapONet](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/15-strategy4-sage.html) | Executa as mesmas sequências em sim e no braço, compara erros por junta e treina compensação de atuação. A página diz que o tutorial é de referência, não exercício hands-on; integração GapONet–GR00T ainda está em desenvolvimento. |
| [16 — Conclusion](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/16-conclusion.html) | Resume randomização, co-treino, Cosmos e análise da diferença de atuação; recomenda ciclos medidos de tentativa e revisão. |
| [Quick Reference](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/quick_reference.html) | Reúne comandos e containers. Os exemplos dão acesso privilegiado a dispositivos e GPU; não são comandos para executar no nosso braço antes das verificações elétricas. |
| [Datasets and Models](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/datasets-and-models.html) | Lista os datasets e checkpoints que a trilha compara. Há uma inconsistência no total de demonstrações reais, detalhada abaixo. |
| [Troubleshooting](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/troubleshooting.html) | Agrupa falhas de motores, USB, câmera, calibração, gravação e avaliação. |

## Vídeos ligados ao curso

A playlist oficial tem quatro vídeos:

1. **It's Time to Build Your Home Robot Lab** (3:30): apresenta o fluxo sim-to-real e o kit físico. Mostra leader e follower como peças distintas do workshop.
2. **Building the SO-101 Workspace** (5:49): mostra o lightbox, o braço, os frascos, a iluminação e a posição da webcam externa.
3. **Solve This Problem With the SO-101** (2:12): explica por que o pick-and-place é mais difícil do que parece; a câmera da garra perde visão quando o frasco é agarrado, e a inserção exige alinhamento fino.
4. **How to Train AI Robots with Isaac Sim and Isaac Lab — Robotics Office Hours** (1:13:08): sessão ampla com demonstração, teleoperação, estratégias e perguntas. As legendas confirmam: operador precisa usar apenas as câmeras; demonstrações devem ser suaves, diretas e variadas; Cosmos muda imagens mantendo a mesma ação; SAGE mede divergência entre sim e hardware. Os apresentadores dizem que a integração SAGE/GapONet com GR00T ainda não existe.

As páginas também contêm GIFs/figuras de teleoperação simulada, dataset, avaliação, calibração e comparação SAGE. Visualmente, o curso deixa clara a cena com o follower, o rack e as duas perspectivas de câmera. A transmissão longa é material de apoio geral, não uma sequência obrigatória de passos para o nosso follower isolado.

## Diferenças e ressalvas encontradas

1. **N1.6 do curso versus N1.7 atual.** As instruções do curso dizem GR00T N1.6, saída em posição articular e horizonte 16. O repositório NVIDIA lista N1.7 como versão mais recente e descreve mudanças no backbone, interfaces e ação relativa de efetuador final. Não se deve copiar comandos/configuração de um para o outro sem adaptar dataset e modalidade.
2. **Representação de ação não é consistente nas páginas.** A página de GR00T descreve ações em juntas; a transmissão fala de políticas relativas. Antes de treinar ou executar um checkpoint, conferir a configuração de modalidade, metadados e espaço de ação daquele modelo.
3. **Quantidade real diverge.** A página 13 e o dataset real publicado indicam 5 episódios reais; a tabela de modelos na página 13 e a página Datasets and Models mencionam 50 demonstrações reais no checkpoint sim+real. O total exato de treino desse checkpoint não fica coerente no material.
4. **O curso não promete política perfeita.** O modelo sim-only reportado tem sucesso moderado no simulador e o foco é comparar estratégias, não atingir uma taxa garantida no hardware.
5. **Requisitos de hardware.** A trilha é um fluxo NVIDIA Isaac/GPU e não uma receita mínima para mover um SO-101. Para começar a aprender o braço, a stack LeRobot/ACT é mais leve; para replicar este workshop inteiro, é preciso acompanhar versões, GPU, containers, duas câmeras, leader e dados.

## Próxima sequência recomendada para o nosso projeto

1. **Fonte do braço ainda exige conferência física.** O texto original deste relatório registrava uma fonte de 12 V/2 A, mas essa informação ficou desatualizada: você informou depois ter uma fonte de 12 V/5 A. Confirme etiqueta, regulação, ficha, polaridade e compatibilidade com a placa antes de conectar.
2. Com a fonte correta, inspecionar alimentação, montagem, cabos, controlador e cortes de emergência; depois identificar IDs e calibrar apenas o follower.
3. Escolher a teleoperação para uma arm: começar por comandos de junta simples ou implementar o caminho do telefone/VR com pose cartesiana, URDF, limites de workspace e IK.
4. Gravar poucas demonstrações de teste e conferir quadro a quadro se imagem, estado e ação estão sincronizados; melhorar a qualidade antes de aumentar quantidade.
5. Treinar ACT como baseline no LeRobot. Testar cada execução em área livre e baixa velocidade antes de deixar a política repetir a tarefa.
6. Só depois comparar VLA/GR00T, começando por um dataset e checkpoint compatíveis com a versão. Para tarefa real com uma câmera de garra, considerar adicionar uma câmera fixa antes de esperar que o modelo veja objetos ocluídos.

## Fontes principais

- [Curso NVIDIA SO-101 completo](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/index.html)
- [ACT no LeRobot](https://huggingface.co/docs/lerobot/act)
- [Teleoperação por telefone no LeRobot](https://huggingface.co/docs/lerobot/en/phone_teleop)
- [Formato de datasets e fluxo LeRobot](https://huggingface.co/docs/lerobot/main/en/index)
- [Releases oficiais do NVIDIA Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T/releases)
- [Dataset real do curso (5 episódios)](https://huggingface.co/datasets/sreetz-nv/so101_teleop_vials_rack_left_real)
- [Playlist oficial de vídeos](https://www.youtube.com/playlist?list=PL2bKqBZg-pzVQspO8-wieuIFctBdz_Tr_)
