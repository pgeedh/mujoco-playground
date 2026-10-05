# mujoco-playground

**Play it:** https://mujoco-playground-alpha.vercel.app

Three robots, three missions, real MuJoCo physics (WebAssembly) in your browser.
Every run is timed and scored, so it doubles as a small manipulation / locomotion /
navigation benchmark.

| Robot | Mission | Controls |
|---|---|---|
| **Unitree G1** humanoid | Dishwasher loading: carry 3 plates down the path | W A S D walk, Space jump, Q / E grab, C camera |
| **Bimanual Franka** (2x Panda) | Cube stacking: red on green, then blue on top | Tab switch arm, W A S D / R F move gripper, Q E wrist, Space gripper |
| **Clearpath Husky on Mars** | Sample return: survey beacon, collect the sample, return to the lander | W S drive, A D steer, hold E to collect, M map, C camera |

**Esc** returns to the mission select. Add `?seed=N` to the URL (or press **New seed**)
for a different scenario.

## Benchmark

Each environment exposes the same interface (`web/src/bench.js`): a seeded reset, a
clock that starts at your first input, a success check, a 0-100 score and a JSON
export (**Export JSON** in the panel).

| Env | Success | Score (max 100) | Seed controls |
|---|---|---|---|
| G1 | all 3 plates in the dishwasher | 30 per plate + up to 10 time bonus - 5 per fall | deterministic |
| Franka | 3-cube stack (stage 1: A on B, stage 2: C on A), stable for 0.6 s | 45 per stage + up to 10 time bonus - 5 per dropped cube | cube positions and yaw |
| Husky | survey beacon, sample collected (hold E 3 s), back at the lander | 30 per phase + up to 10 time bonus - 2 per boulder hit - 5 per rollover | which of 4 sample sites |

A result file looks like `{ "env": "rover", "seed": 3, "success": true, "time_s": 92.4, "score": 98, "metrics": {...} }`.
The Franka stack check follows [robosuite](https://github.com/ARISE-Initiative/robosuite)'s
`Stack` environment (top cube resting on the bottom cube and released). The
Panda model is from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
and the Husky meshes are Clearpath's, BSD-3 (`web/assets/scenes/*/LICENSE_*`).

### Using World Labs worlds

Drop a world exported from World Labs (Marble) or any glTF into
`web/assets/worlds/<env id>/` (`g1`, `franka` or `rover`) with a `world.json`:

```json
{ "file": "scene.glb", "position": [0, 0, 0], "rotation": [0, 0, 0], "scale": 1, "hide": ["tree1", "tree2"] }
```

It is shown as the visual backdrop (three.js coordinates, y up) and `hide` lists
MuJoCo body names whose stand-in visuals should be hidden. Physics still comes from
the MuJoCo scene, so match the collision geometry you need in the scene XML.

## G1: what's real vs. scripted

- **Walking (W/A/S/D)**: a real pretrained locomotion policy from
  [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)
  (an LSTM, converted from its original TorchScript checkpoint to ONNX —
  see `scripts/export_policy.py`, bit-verified identical output), run via
  `onnxruntime-web`. It genuinely balances and walks; this isn't a canned
  animation.
- **Grabbing (Q = right hand, E = left hand)**: no pretrained grasping
  policy exists publicly for the G1's hands, and MuJoCo's official WASM
  bindings have a bug that makes runtime equality-constraint toggling
  (`data.eq_active`) unusable. Grabbing is done by directly snapping the
  held object's position to the wrist every physics step — a kinematic
  hack, not physical grasping. The model does use the real hands-enabled
  G1 variant (`g1_with_hands.xml`, 14 extra finger joints) and the fingers
  genuinely close/open via their own position actuators in sync with grab
  state, so it looks like a real grasp — but the object's placement itself
  is still the kinematic snap, not held by finger contact/friction.
- **Jump (Space)**: no pretrained jump/parkour skill exists publicly for
  the G1 either. It's a scripted crouch-then-extend leg trajectory that
  briefly overrides the walking policy's output.
- **Auto-reset**: if the robot stays collapsed (pelvis near the floor) for
  over ~0.6s, only the robot itself resets back to spawn (pose + the
  policy's internal state) — not the whole world, so already-delivered
  task progress and other props are untouched. There's no trained
  recovery/get-up skill, so restarting the robot is the only option.
- **Camera (C)**: toggles between third-person (free orbit) and first-person
  (glued to the torso, looking the way it's facing) — this part's just
  normal Three.js camera work, nothing ML-related.

## Sandbox and optional goal

The park is a free-roam sandbox: grab plates, balls and crates with Q / E.
The original goal is still there if you want one: load 3 plates into the dishwasher. Plates start stacked on a counter right
in front of spawn; the dishwasher is a short walk further down the path.
Carry them over with Q (right hand) / E (left hand) — one at a time is more
reliable than two at once (see rough edges below). Progress is shown
onscreen and is sticky once a plate is delivered, surviving a fall/reset.

Note: this pretrained walking policy wasn't trained for long sustained
walks — even completely empty-handed, it will occasionally
stumble and fall after a few seconds of continuous walking, unrelated to
carrying anything or the scene layout. That's why auto-reset exists; it's
a real characteristic of the checkpoint, not a bug in this integration.

## Local setup

**Python side** (verifies the MuJoCo model/scene independent of the browser):
```
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python scripts/test_world.py
```
Loads `models/unitree_g1/world.xml`, holds a standing pose for one
simulated second to confirm the model balances, and writes a render to
`scripts/world_preview.png`.

**Browser demo**:
```
cd web
npm install
python3 -m http.server 8080   # any static file server works
```
Then open `http://localhost:8080/index.html`.

To regenerate the ONNX policy from the original checkpoint:
```
git clone --depth 1 https://github.com/unitreerobotics/unitree_rl_gym /tmp/unitree_rl_gym
python scripts/export_policy.py /tmp/unitree_rl_gym/deploy/pre_train/g1/motion.pt \
    web/assets/policy/g1_walk_policy.onnx
```

## Layout

- `models/unitree_g1/` — vendored G1 MJCF + meshes from
  [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)
  (BSD-3-Clause, see `LICENSE` in that folder) — `g1_with_hands.xml` is the
  one actually used (`g1.xml`, without fingers, is kept for reference) —
  plus `world.xml`, the task scene built on top of it.
- `web/` — the browser app: MuJoCo WASM (`@mujoco/mujoco`) + Three.js for
  physics/rendering (trimmed down from
  [zalo/mujoco_wasm](https://github.com/zalo/mujoco_wasm)'s demo scaffold —
  its other bundled example scenes/robots were removed, keeping only what
  this app actually uses), plus `src/g1Control.js` (all G1-specific
  control: the walking policy inference loop, jump, grab, fall detection,
  task tracking) and `src/sceneLoader.js` (downloads a scene's assets into
  MuJoCo's virtual filesystem); the other robots live in `src/frankaControl.js`
  and `src/marsControl.js`, the registry in `src/envs.js`, scoring in `src/bench.js`.
- `scripts/` — `build_bimanual.py` and `build_mars.py` generate the Franka and Mars scenes, `make_manifests.py` lists each scene's files for the browser; plus local Python dev/test scripts (CPU-only, no GPU needed) and
  the ONNX export script.

## Known rough edges

- Grab reach is intentionally generous (1.6m) since the arms are held at a
  fixed relaxed pose rather than actively reaching (no IK) — it's closer to
  "stand near it and grab" than a precise hand-touch.
- Walking backward drifts more than forward (an artifact of the policy's
  training, not this integration).
- Carrying two plates (one per hand) at once noticeably increases how often
  the robot stumbles compared to carrying one — the mechanism is still a
  kinematic snap-to-wrist per plate, so it's likely the doubled asymmetric
  arm-position deviation from the pose the policy expects, but this hasn't
  been root-caused further. One at a time is more reliable.
- Solid, robot-collidable furniture directly in the walking path reliably
  trips this policy (it was only trained on flat ground) — everything in
  this scene the robot walks near is deliberately non-colliding
  (`contype`/`conaffinity` 0, or a separate group like the counter/plates)
  for exactly this reason. Worth remembering if you add new scene geometry.
## Deploying (Vercel)

The repo-root `vercel.json` builds `web/` (and `web/vercel.json` covers the case where the project's Root Directory is set to `web`). The build runs
`npm run build` (`web/scripts/build.mjs`), which copies the runtime
dependencies out of `node_modules` into `web/dist/`, served as a static site.
Test locally with `cd web && npm install && npm run build && cd dist && python3 -m http.server 8080`.
