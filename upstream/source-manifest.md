# Upstream source manifest

| Field | Value |
|---|---|
| Canonical repository | https://github.com/TheRobotStudio/SO-ARM100 |
| User fork | https://github.com/psampaioc/SO-ARM100 |
| Local clone | `upstream/SO-ARM100` |
| Origin remote | `https://github.com/psampaioc/SO-ARM100.git` |
| Upstream remote | `https://github.com/TheRobotStudio/SO-ARM100.git` |
| Branch | `main` |
| Commit | `7629d2ad9853d10fb903093a33ef6114099d97e5` |
| Commit date | 2026-08-05T12:21:21+03:00 |
| Commit subject | Add Robonine as worldwide supplier to Kits list (#164) |
| Variant selected | SO-101 follower only |
| Leader | Excluded |
| Local copy policy | Tracked files in the clone remain untouched; local-only print artifacts are listed below |
| Adaptation boundary | Project-specific files will live under `project/` |

## Source files copied or referenced

- `upstream/BOM.original.md` - verbatim follower BOM section from the upstream README.
- `upstream/SO-ARM100/README.md` - complete upstream README and current documentation.
- `upstream/SO-ARM100/LICENSE` - upstream license.
- `upstream/SO-ARM100/STL/SO101/Follower/Ender_Follower_SO101.stl` - official combined STL for approximately 220 x 220 mm beds.
- `upstream/SO-ARM100/STL/SO101/Follower/Prusa_Follower_SO101.stl` - official combined STL for approximately 205 x 250 mm beds.
- `upstream/SO-ARM100/STL/Gauges/Gauge_0.STL` - official fit gauge.
- `upstream/SO-ARM100/STEP/SO101/` - official SO-101 STEP files for reference/adaptation.
- `https://huggingface.co/docs/lerobot/so101` - current SO-101 assembly and calibration guide.

## Integrity checks

- The clone has separate `origin` and `upstream` remotes.
- The upstream Git checkout remains at the commit above. Its working tree currently contains two untracked local print artifacts: `STL/SO101/Follower/so-101_impressao_CR.zip` and `STL/SO101/Follower/so-101_impressao_CR/`.
- These local-only artifacts remain untouched inside the source clone; they are not upstream files and are excluded from the root project Git repository.
