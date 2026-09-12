"""Sanity check: load the G1 + obstacles world, hold the standing pose under
gravity with simple PD control, and render a frame to confirm the scene is
built correctly."""
import os
import sys

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(ROOT, "models", "unitree_g1", "world.xml")


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    stand_qpos = data.qpos[7:].copy()  # joint angles only, skip free joint

    print(f"nq={model.nq} nv={model.nv} nu={model.nu} bodies={model.nbody}")
    print(f"obstacles present: {[model.body(i).name for i in range(model.nbody) if 'obstacle' in model.body(i).name]}")

    # g1.xml's actuators are <position> servos with built-in kp/damping, so ctrl
    # is just the target joint angle — hold the standing keyframe pose directly.
    data.ctrl[:] = stand_qpos[: model.nu]
    for step in range(500):
        mujoco.mj_step(model, data)

    pelvis_z = data.qpos[2]
    print(f"pelvis height after 500 steps ({500*model.opt.timestep:.2f}s sim time): {pelvis_z:.3f} m")
    assert pelvis_z > 0.5, "robot fell over — PD hold failed"

    renderer = mujoco.Renderer(model, height=480, width=640)
    renderer.update_scene(data, camera=-1)
    img = renderer.render()
    out_path = os.path.join(ROOT, "scripts", "world_preview.png")
    import imageio.v2 as imageio
    imageio.imwrite(out_path, img)
    print(f"wrote preview render to {out_path}")


if __name__ == "__main__":
    sys.exit(main())
