# Proveniência da teleoperação SO-101

## Repositórios de referência

Estes checkouts locais ficaram em `/tmp/desk-arm-reference-audit-20261107/`, fora do repositório e do runtime; já não estão disponíveis para nova comparação linha a linha. A auditoria do código ativo e dos registos desta tarefa indica que nenhum código-fonte destes projetos foi copiado para o runtime.

| Projeto | Origem e commit fixo | Uso previsto / licença no commit auditado |
|---|---|---|
| AFK-Kireeti/Lerobot-SO101 | https://github.com/AFK-Kireeti/Lerobot-SO101/tree/32c561abd05354fb51aae2bdf0e74df8b0df9ff2 | Comparar três métodos (ROS/webcam, Pi com IMU e ArUco); sem LICENSE na raiz, referência conceitual. |
| ABMI-software/hand_controlLerobot | https://github.com/ABMI-software/hand_controlLerobot/tree/52c7f0d926d0f66107ae91b94ceb900766f3009d | Geometria 3D da mão e reancoragem; Apache-2.0, mas nenhum código copiado para o runtime atual. |
| sruthihsr/SO101-Gesture-Teleoperation | https://github.com/sruthihsr/SO101-Gesture-Teleoperation/tree/ac5468b603a65c556634893ef8564873206734c4 | Estados, TCP e Jacobiana posicional; sem LICENSE na raiz, portanto referência conceitual. |
| BrosnanYuen/hand_pose_lerobot_teleop | https://github.com/BrosnanYuen/hand_pose_lerobot_teleop/tree/425a4fba62b6c7c07fc3968a18d00da99bc193f2 | Comparado como alternativa LeRobot hand-pose; não integrado nem copiado. Licença completa não confirmada na revisão anterior. |

Os três SHAs e URLs foram verificados antes da preparação. Referências não são dependência de build ou runtime.

## Modelo cinemático do runtime atual

O runtime usa `teleop/models/so101/so101_new_calib.urdf` e os meshes referenciados na mesma pasta. Origem e commit: TheRobotStudio/SO-ARM100 `Simulation/SO101/so101_new_calib.urdf`, commit `7629d2ad9853d10fb903093a33ef6114099d97e5`. Cópia, licença Apache-2.0 e SHA-256 estão registrados em [teleop/models/so101/PROVENANCE.md](models/so101/PROVENANCE.md). A IK de runtime é `lerobot.model.kinematics.RobotKinematics` do LeRobot 0.7.0/PlaCo; o solver MuJoCo abaixo pertence ao arquivo ROS legado preservado e não é importado pela teleoperação atual.

## Modelo MuJoCo legado preservado (fora do runtime atual)

Origem: [TheRobotStudio/SO-ARM100, Simulation/SO101](https://github.com/TheRobotStudio/SO-ARM100/tree/7629d2ad9853d10fb903093a33ef6114099d97e5/Simulation/SO101), commit `7629d2ad9853d10fb903093a33ef6114099d97e5`. Os arquivos abaixo foram copiados sem alteração para este pacote; os SHA-256 foram conferidos contra a origem local. A licença Apache-2.0 está em `ros2/desk_arm_description/LICENSE`.

| Arquivo | SHA-256 |
|---|---|
| `ros2/desk_arm_description/model/scene.xml` | `3b79a253a742f55ff0b16682173609229560d97744be472a238ea2e0a6a31ef6` |
| `ros2/desk_arm_description/model/so101_new_calib.xml` | `d75253eb568e8a7214db9c631ab7bed4217f608a26f7276ebe9a7636cac82580` |
| `model/assets/base_motor_holder_so101_v1.stl` | `8cd2f241037ea377af1191fffe0dd9d9006beea6dcc48543660ed41647072424` |
| `model/assets/base_so101_v2.stl` | `bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d` |
| `model/assets/motor_holder_so101_base_v1.stl` | `31242ae6fb59d8b15c66617b88ad8e9bded62d57c35d11c0c43a70d2f4caa95b` |
| `model/assets/motor_holder_so101_wrist_v1.stl` | `887f92e6013cb64ea3a1ab8675e92da1e0beacfd5e001f972523540545e08011` |
| `model/assets/moving_jaw_so101_v1.stl` | `785a9dded2f474bc1d869e0d3dae398a3dcd9c0c345640040472210d2861fa9d` |
| `model/assets/rotation_pitch_so101_v1.stl` | `9be900cc2a2bf718102841ef82ef8d2873842427648092c8ed2ca1e2ef4ffa34` |
| `model/assets/sts3215_03a_no_horn_v1.stl` | `75ef3781b752e4065891aea855e34dc161a38a549549cd0970cedd07eae6f887` |
| `model/assets/sts3215_03a_v1.stl` | `a37c871fb502483ab96c256baf457d36f2e97afc9205313d9c5ab275ef941cd0` |
| `model/assets/under_arm_so101_v1.stl` | `d01d1f2de365651dcad9d6669e94ff87ff7652b5bb2d10752a66a456a86dbc71` |
| `model/assets/upper_arm_so101_v1.stl` | `475056e03a17e71919b82fd88ab9a0b898ab50164f2a7943652a6b2941bb2d4f` |
| `model/assets/waveshare_mounting_plate_so101_v2.stl` | `e197e24005a07d01bbc06a8c42311664eaeda415bf859f68fa247884d0f1a6e9` |
| `model/assets/wrist_roll_follower_so101_v1.stl` | `4b17b410a12d64ec39554abc3e8054d8a97384b2dc4a8d95a5ecb2a93670f5f4` |
| `model/assets/wrist_roll_pitch_so101_v2.stl` | `6c7ec5525b4d8b9e397a30ab4bb0037156a5d5f38a4adf2c7d943d6c56eda5ae` |

## Modelo de mão MediaPipe

Obtido de https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task e mantido localmente em `data/models/hand_landmarker.task` (ignorado pelo Git).

SHA-256: `fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1`.

## Runtime ativo — 2026-11-09

- Base OCI: Ubuntu 24.04 (`ubuntu:24.04`), Python 3.12.
- LeRobot 0.7.0 com extras `feetech` e `kinematics`, PlaCo 0.9.15, MediaPipe 0.10.35 e PyTorch CPU 2.15.0.
- Imagem: `desk-arm-lerobot:0.7.0-single-process`; digest local `sha256:7a84a12a4316c50715091201fab169c063592135faf8e5f6a179b789d36580e5`.
- Na primeira validação, `pip check`, imports, URDF, FK/IK, calibração somente leitura e 30 testes offline passaram; um preview abriu `/dev/video0` sem serial. Após a correção do detector/limites em 2026-11-09, `pip check` passou e os 33 testes offline passaram; o último passe não abriu câmera, não acessou serial e não comandou servos.
- A imagem antiga `desk-arm-lerobot:jazzy-0.7.0` foi removida depois da migração. Os arquivos ROS/MuJoCo continuam preservados para histórico, mas não são dependências do comando `python -m teleop.hand_controller`.

## Ambiente de execução anterior — ROS Jazzy

- Base OCI: `ros:jazzy-ros-base-noble`, resolvida no build para `sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617`.
- Build Python: 3.12.3; ROS 2 Jazzy; LeRobot 0.7.0; MediaPipe 0.10.35; MuJoCo 3.17.0; setuptools 79.0.1.
- `pip check`, imports conjuntos de ROS/LeRobot/MediaPipe/MuJoCo/OpenCV, `colcon build` do pacote próprio e carga do modelo (seis juntas e seis atuadores) passaram em container temporário em 2026-11-07.
- O container persistente antigo usava `desk-arm-lerobot:jazzy-0.7.0`; essa instância e imagem foram removidas após verificar que não havia dados de projeto na camada gravável e nenhum outro container a usava.
- O pacote da mão e o modelo SO-101 foram obtidos de fontes oficiais; os três repositórios de referência continuam fora do Git e não são dependências de execução.
