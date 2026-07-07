# Open Duck Mini v2 — vendored assets

The contents of this directory derive from the **Open Duck Mini** project by Antoine Pirrone and
contributors, licensed under the **Apache License 2.0**:

- <https://github.com/apirrone/Open_Duck_Mini> — robot design, URDF, and the trained walking policy
- <https://github.com/apirrone/Open_Duck_Playground> — the MuJoCo RL training environment

| Path | What it is |
|---|---|
| `robot/` | The robot converted from the project's URDF to USD with `tools\import_duck_urdf.py` (Isaac Sim's URDF importer); geometry and joint definitions are the project's, drive gains match the training MJCF |
| `policy/BEST_WALK_ONNX_2.onnx` | The project's exported walking-policy network, unmodified |

See the source repositories for the full license text and original notices. This repository's own
license is in [LICENSE](../../LICENSE) at the root.
