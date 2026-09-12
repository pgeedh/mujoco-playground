# humanoid-mujoco

A real Unitree G1 humanoid, simulated in real MuJoCo physics, jumping over obstacles.

## Status

**Stage 1 (done):** a MuJoCo world with the real 29-DOF Unitree G1 model
([mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie), BSD-3-Clause)
standing in a scene with two obstacles — a low barrier to jump over and a
step/ramp to climb.

**Next stages (planned):**
- WASD + spacebar control, driven by a real trained locomotion policy (not
  hand-scripted control) — sourced from
  [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) and/or
  NVIDIA's [MotionBricks](https://github.com/NVlabs/GR00T-WholeBodyControl).
- Port the physics + policy to run entirely client-side in the browser
  (MuJoCo compiled to WebAssembly + the policy converted to ONNX, run via
  `onnxruntime-web`) so it can be embedded on a website via Vercel.
- Richer park environment.

Jumping specifically has no publicly available pretrained policy for the G1
today (confirmed via research — see project notes); it will need to be a
scripted/heuristic behavior layered on the walking policy, or a
from-scratch trained skill.

## Local setup

```
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python scripts/test_world.py
```

This loads `models/unitree_g1/world.xml`, holds the standing pose for one
simulated second to confirm the model balances, and writes a render to
`scripts/world_preview.png`.

## Layout

- `models/unitree_g1/` — vendored G1 MJCF + meshes from mujoco_menagerie
  (see `LICENSE` in that folder), plus `world.xml`, the park/obstacle scene
  built on top of it.
- `scripts/` — local test/dev scripts (Python, CPU-only, no GPU required
  for this stage).
