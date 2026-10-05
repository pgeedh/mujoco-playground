// Unitree G1 humanoid with a real pretrained walking policy.
// Legs: the actual locomotion policy from unitree_rl_gym (LSTM, converted from TorchScript to ONNX,
// see scripts/g1/export_policy.py), run client-side with onnxruntime-web. Waist and arms are held at a
// relaxed pose (the checkpoint never learned to control them). Jump is a scripted crouch-and-extend
// that briefly overrides the legs: no pretrained jump skill exists publicly for the G1.
// The mission (nuclear.js) plugs in through `this.mission`: it is stepped with the physics, updated
// every frame and supplies the benchmark metrics.
import { PlantMission } from "./nuclear.js";

const ort = window.ort;

const CFG = {
  kpsLeg: [100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40],
  kdsLeg: [2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2],
  legDefaultAngles: [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0],
  // waist (3) + left arm (7), from g1_with_hands.xml's "stand" keyframe.
  // Actuator layout (43 total, see g1_with_hands.xml): legs 0-11 (motors,
  // policy-controlled), waist 12-14, left arm 15-21, left hand 22-28,
  // right arm 29-35, right hand 36-42.
  waistLeftArmAngles: [0, 0, 0, 0.2, 0.2, 0, 1.28, 0, 0, 0],
  rightArmAngles: [0.2, -0.2, 0, 1.28, 0, 0, 0],
  // Finger actuator order differs per hand (mirrors the joint definitions in g1_with_hands.xml):
  // left is [thumb0,thumb1,thumb2,middle0,middle1,index0,index1]; right swaps index/middle.
  // The mission never grasps, so the hands simply stay in their relaxed open pose.
  leftHandOpen: [0, 1.05, 0, 0, 0, 0, 0],
  leftHandClosed: [0, 1.05, 1.5, -1.3, -1.6, -1.3, -1.6],
  rightHandOpen: [0, -1.05, 0, 0, 0, 0, 0],
  rightHandClosed: [0, -1.05, 1.5, 1.3, 1.6, 1.3, 1.6],
  angVelScale: 0.25,
  dofPosScale: 1.0,
  dofVelScale: 0.05,
  actionScale: 0.25,
  cmdScale: [2.0, 2.0, 0.25],
  numActions: 12,
  numObs: 47,
  controlDecimation: 10, // policy runs at simDt*10 = 50 Hz
  period: 0.8,
  hiddenSize: 64,
};

const FALL_PELVIS_HEIGHT = 0.45; // meters; standing height is ~0.77-0.79m
const FALL_RESET_DELAY = 0.6; // seconds sustained below that height before auto-reset

function quatToGravity(qw, qx, qy, qz) {
  return [
    2 * (-qz * qx + qw * qy),
    -2 * (qz * qy + qw * qx),
    1 - 2 * (qw * qw + qz * qz),
  ];
}

export class G1Controller {
  constructor() {
    this.session = null;
    this.ready = false;
    this.inferencePending = false;

    this.h = new Float32Array(CFG.hiddenSize);
    this.c = new Float32Array(CFG.hiddenSize);
    this.prevAction = new Float32Array(CFG.numActions);
    this.targetDofPos = Float32Array.from(CFG.legDefaultAngles);

    this.stepCounter = 0;
    this.cmd = [0, 0, 0];
    this.keys = {};
    this._prevKeys = {};

    this.jumpState = "idle"; // idle -> crouch -> extend -> idle
    this.jumpTimer = 0;
    this.jumpOverride = null;

    this.fallTimer = 0;
    this.falls = 0;
    this.anyInput = false;
    this._lastStand = null;
    this.mission = new PlantMission();

    window.addEventListener("keydown", (e) => {
      this.keys[e.code] = true;
      if (["KeyW", "KeyA", "KeyS", "KeyD", "Space"].includes(e.code)) this.anyInput = true;
    });
    window.addEventListener("keyup", (e) => { this.keys[e.code] = false; });
  }

  async load() {
    ort.env.wasm.wasmPaths = new URL("../../../node_modules/onnxruntime-web/dist/", import.meta.url).href;
    ort.env.wasm.numThreads = 1;
    this.session = await ort.InferenceSession.create(
      new URL("../../../assets/policy/g1_walk_policy.onnx", import.meta.url).href
    );
    await this.mission.load();
    this.ready = true;
  }

  /** Resolve body/joint addresses once the model is loaded. */
  bindModel(model, mujoco, ctx) {
    this.model = model;
    this.mujoco = mujoco;
    // Every qpos/qvel entry belongs to the robot in this scene, so a fall reset only has to restore those.
    this.robotNq = model.nq; this.robotNv = model.nv;
    this.robotQpos0 = Float32Array.from(model.qpos0.slice(0, this.robotNq));
    this.mission.bind(model, mujoco, ctx, this);
  }

  _updateCommand() {
    let vx = 0, wz = 0;
    if (this.keys["KeyW"]) vx += 1;
    if (this.keys["KeyS"]) vx -= 1;
    if (this.keys["KeyA"]) wz += 1;
    if (this.keys["KeyD"]) wz -= 1;
    this.cmd = [vx, 0, wz];
  }

  _buildObs(data) {
    const obs = new Float32Array(CFG.numObs);
    const qw = data.qpos[3], qx = data.qpos[4], qy = data.qpos[5], qz = data.qpos[6];
    const gravity = quatToGravity(qw, qx, qy, qz);
    const omega = [data.qvel[3] * CFG.angVelScale, data.qvel[4] * CFG.angVelScale, data.qvel[5] * CFG.angVelScale];

    obs[0] = omega[0]; obs[1] = omega[1]; obs[2] = omega[2];
    obs[3] = gravity[0]; obs[4] = gravity[1]; obs[5] = gravity[2];
    obs[6] = this.cmd[0] * CFG.cmdScale[0];
    obs[7] = this.cmd[1] * CFG.cmdScale[1];
    obs[8] = this.cmd[2] * CFG.cmdScale[2];
    for (let i = 0; i < 12; i++) {
      obs[9 + i] = (data.qpos[7 + i] - CFG.legDefaultAngles[i]) * CFG.dofPosScale;
      obs[21 + i] = data.qvel[6 + i] * CFG.dofVelScale;
      obs[33 + i] = this.prevAction[i];
    }
    const period = CFG.period;
    const count = this.stepCounter * this.model.opt.timestep;
    const phase = (count % period) / period;
    obs[45] = Math.sin(2 * Math.PI * phase);
    obs[46] = Math.cos(2 * Math.PI * phase);
    return obs;
  }

  async _runPolicy(data) {
    if (!this.ready || this.inferencePending) return;
    this.inferencePending = true;
    try {
      const obs = this._buildObs(data);
      const feeds = {
        obs: new ort.Tensor("float32", obs, [1, CFG.numObs]),
        h_in: new ort.Tensor("float32", this.h, [1, 1, CFG.hiddenSize]),
        c_in: new ort.Tensor("float32", this.c, [1, 1, CFG.hiddenSize]),
      };
      const results = await this.session.run(feeds);
      this.prevAction = Float32Array.from(results.action.data);
      this.h = Float32Array.from(results.h_out.data);
      this.c = Float32Array.from(results.c_out.data);
      const target = new Float32Array(CFG.numActions);
      for (let i = 0; i < CFG.numActions; i++) {
        target[i] = this.prevAction[i] * CFG.actionScale + CFG.legDefaultAngles[i];
      }
      this.targetDofPos = target;
    } finally {
      this.inferencePending = false;
    }
  }

  /** Scripted jump: not a trained skill (none exists publicly for G1) —
   * a quick crouch-then-extend leg trajectory that briefly overrides the
   * policy's leg targets to impart an upward impulse. */
  _updateJump(dt) {
    const spaceDown = !!this.keys["Space"];
    const spaceEdge = spaceDown && !this._prevKeys["Space"];
    this._prevKeys["Space"] = spaceDown;

    if (spaceEdge && this.jumpState === "idle") {
      this.jumpState = "crouch";
      this.jumpTimer = 0;
    }

    if (this.jumpState === "crouch") {
      this.jumpTimer += dt;
      this.jumpOverride = [-0.6, 0, 0, 1.1, -0.5, 0, -0.6, 0, 0, 1.1, -0.5, 0];
      if (this.jumpTimer > 0.18) { this.jumpState = "extend"; this.jumpTimer = 0; }
    } else if (this.jumpState === "extend") {
      this.jumpTimer += dt;
      this.jumpOverride = [0.2, 0, 0, -0.1, 0.15, 0, 0.2, 0, 0, -0.1, 0.15, 0];
      if (this.jumpTimer > 0.10) { this.jumpState = "idle"; this.jumpOverride = null; }
    } else {
      this.jumpOverride = null;
    }
  }

  /** If the robot has been collapsed (pelvis near the floor) for longer
   * than FALL_RESET_DELAY, reset the whole sim back to its start state —
   * no trained recovery/get-up skill exists, so the only sensible thing to
   * do after a fall is start over. */
  _checkFall(data, dt) {
    const pelvisZ = data.qpos[2];
    if (pelvisZ < FALL_PELVIS_HEIGHT) {
      this.fallTimer += dt;
    } else {
      this.fallTimer = 0;
    }
    if (this.fallTimer > FALL_RESET_DELAY) {
      this.fallTimer = 0;
      this._resetAfterFall(data);
    }
  }

  _resetAfterFall(data) {
    console.log("[reset] robot fell over: standing it back up where it was");
    this.falls++;
    const keep = this._lastStand;                   // last upright x, y and heading
    data.qpos.set(this.robotQpos0, 0);
    if (keep) {
      data.qpos[0] = keep.x; data.qpos[1] = keep.y;
      data.qpos[3] = Math.cos(keep.yaw / 2); data.qpos[4] = 0; data.qpos[5] = 0; data.qpos[6] = Math.sin(keep.yaw / 2);
    }
    for (let i = 0; i < this.robotNv; i++) data.qvel[i] = 0;
    this.mujoco.mj_forward(this.model, data);
    this.h.fill(0); this.c.fill(0); this.prevAction.fill(0);
    this.targetDofPos = Float32Array.from(CFG.legDefaultAngles);
    this.stepCounter = 0; this.cmd = [0, 0, 0];
    this.jumpState = "idle"; this.jumpTimer = 0; this.jumpOverride = null;
  }

  /** Call once per physics substep (mj_step), before stepping. */
  beforeStep(data, dt) {
    if (this.mission.dead) {                        // dose limit exceeded: the robot goes limp
      data.ctrl.fill(0); return;
    }
    if (data.qpos[2] > 0.7) {
      const qw = data.qpos[3], qz = data.qpos[6], qx = data.qpos[4], qy = data.qpos[5];
      this._lastStand = { x: data.qpos[0], y: data.qpos[1], yaw: Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)) };
    }
    this._checkFall(data, dt);
    this.stepCounter++;
    if (this.stepCounter % CFG.controlDecimation === 0) {
      this._updateCommand();
      this._runPolicy(data); // fire-and-forget; ~sub-ms on this tiny net
    }
    this._updateJump(dt);

    const legTarget = this.jumpOverride || this.targetDofPos;
    for (let i = 0; i < 12; i++) {
      const q = data.qpos[7 + i];
      const dq = data.qvel[6 + i];
      data.ctrl[i] = CFG.kpsLeg[i] * (legTarget[i] - q) - CFG.kdsLeg[i] * dq;
    }
    // Actuator layout: waist+left-arm 12-21, left hand 22-28, right arm 29-35, right hand 36-42.
    for (let i = 0; i < CFG.waistLeftArmAngles.length; i++) data.ctrl[12 + i] = CFG.waistLeftArmAngles[i];
    for (let i = 0; i < CFG.leftHandOpen.length; i++) data.ctrl[22 + i] = CFG.leftHandOpen[i];
    for (let i = 0; i < CFG.rightArmAngles.length; i++) data.ctrl[29 + i] = CFG.rightArmAngles[i];
    for (let i = 0; i < CFG.rightHandOpen.length; i++) data.ctrl[36 + i] = CFG.rightHandOpen[i];
    this.mission.step(data, dt);
  }

  /** Call once per rendered frame (not per physics substep). */
  update(data) {
    const now = performance.now(), dt = this._t ? Math.min(0.1, (now - this._t) / 1000) : 0.016; this._t = now;
    this.mission.update(data, dt);
  }

  dispose() { this.mission.dispose(); }

  // ---- benchmark interface (see core/bench.js) -------------------------------
  hasInput() { return this.anyInput; }

  /** Back to the start: robot at spawn, mission state cleared. */
  reset(data, seed) {
    data.qpos.set(this.model.qpos0.subarray(0, this.model.nq));
    data.qvel.fill(0);
    this.mujoco.mj_forward(this.model, data);
    this.h.fill(0); this.c.fill(0); this.prevAction.fill(0);
    this.targetDofPos = Float32Array.from(CFG.legDefaultAngles);
    this.stepCounter = 0; this.cmd = [0, 0, 0];
    this.jumpState = "idle"; this.jumpTimer = 0; this.jumpOverride = null;
    this.fallTimer = 0; this.falls = 0; this.anyInput = false; this._lastStand = null;
    this.mission.reset(data, seed);
  }

  isSuccess() { return this.mission.isSuccess(); }
  isFinished() { return this.mission.isFinished(); }
  isFailed() { return this.mission.isFailed(); }
  failReason() { return this.mission.failReason(); }
  getMetrics() { return this.mission.getMetrics(); }
  score(time, limit, success) { return this.mission.score(time, limit, success); }
}
