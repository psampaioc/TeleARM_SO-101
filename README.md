# TeleARM — SO-101 hand teleoperation

## Início rápido

Num terminal zsh, cria e entra no container temporário com:

```bash
deskarm
```

Se o terminal já estava aberto quando o comando foi configurado, executa primeiro `source ~/.zshrc` ou abre um novo terminal. Alternativamente, a partir da pasta do projeto:

```bash
docker compose run --rm --name so101-hand lerobot bash
```

Dentro do container, inicia a teleoperação:

```bash
cd /workspace/Brosnantest
python -B hand_camera_teleop.py
```

Este comando liga e pode mover o braço físico. Consulta o [guia completo e as notas de segurança](docs/README.md). Pressiona `Esc` na janela para terminar. Para abrir outro terminal no mesmo container, usa no host:

```bash
docker exec -it so101-hand bash
```

Ao executar `exit` no primeiro terminal, o container temporário termina e é removido. Os ficheiros em `/workspace` e `/workspace/data` permanecem no host.
