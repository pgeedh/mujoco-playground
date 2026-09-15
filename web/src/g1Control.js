// Real-time control for the G1: legs are driven by the actual pretrained
// walking policy from unitree_rl_gym (converted from TorchScript to ONNX,
// see scripts/export_policy.py), run client-side via onnxruntime-web.
// Waist/arms are held at a fixed relaxed pose (the checkpoint never learned
// to control them). Fingers open/close based on grab state, for a visual
// close-the-hand motion — this is NOT a physics grasp (see GRABBABLE_BODIES
// handling below): the object still snaps to the wrist kinematically, the
// fingers are just cosmetic. Jump and grab are NOT learned skills — no
// pretrained policy for either exists publicly for the G1 — they're
// scripted/heuristic behaviors layered on top, described inline below.

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
  // Finger actuator order differs per hand (mirrors the joint definitions
  // in g1_with_hands.xml): left is [thumb0,thumb1,thumb2,middle0,middle1,
  // index0,index1]; right swaps index/middle: [thumb0,thumb1,thumb2,
  // index0,index1,middle0,middle1]. "Open" matches the stand keyframe's
  // relaxed hand; "closed" curls the fingers into a loose grip. There's no
  // real force-closure grasp here (see g1Control.js's grab code) — this is
  // purely a visual finger-close synced to the same kinematic snap-to-wrist
  // that was already carrying the object, per the user's choice to keep the
  // reliable snap rather than risk full contact-based holding.
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

// Arms are held at a fixed relaxed pose (not actively reaching — that would
// need real IK), so the wrist sits close to the hip rather than extended
// toward objects. GRAB_RANGE is generous to compensate: "standing near
// something" counts as reach, since the hand can't stretch out for it.
const GRAB_RANGE = 1.6; // meters, wrist-to-prop distance to allow grabbing
const GRABBABLE_BODIES = ["grab_plate1", "grab_plate2", "grab_plate3"];

// Task: load every plate into the dishwasher.
const TASK_ZONES = {
  grab_plate1: "dishwasher",
  grab_plate2: "dishwasher",
  grab_plate3: "dishwasher",
};
const ZONE_RADIUS = 0.6; // meters, roughly the dishwasher cavity's footprint

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

    this.heldByRightHand = null; // grab_plate1 | grab_plate2 | grab_plate3 | null
    this.heldByLeftHand = null;

    this.fallTimer = 0;

    window.addEventListener("keydown", (e) => { this.keys[e.code] = true; });
    window.addEventListener("keyup", (e) => { this.keys[e.code] = false; });
  }

  async load() {
    ort.env.wasm.wasmPaths = new URL("../node_modules/onnxruntime-web/dist/", import.meta.url).href;
    ort.env.wasm.numThreads = 1;
    this.session = await ort.InferenceSession.create(
      new URL("../assets/policy/g1_walk_policy.onnx", import.meta.url).href
    );
    this.ready = true;
  }

  /** Resolve body/joint addresses once the model is loaded. */
  bindModel(model, mujoco) {
    this.model = model;
    this.mujoco = mujoco;
    const nameId = (type, name) => mujoco.mj_name2id(model, type, name);
    this.rightWristBody = nameId(mujoco.mjtObj.mjOBJ_BODY.value, "right_wrist_yaw_link");
    this.leftWristBody = nameId(mujoco.mjtObj.mjOBJ_BODY.value, "left_wrist_yaw_link");
    // Grabbing is done by directly overwriting each prop's free-joint qpos
    // to follow the wrist every step (kinematic snap), NOT MuJoCo equality
    // constraints: the official WASM bindings' `data.eq_active` getter
    // throws (a bug in their embind glue for boolean arrays), so runtime
    // constraint toggling isn't usable here.
    this.propBodies = {};
    this.propJoints = {};
    for (const name of GRABBABLE_BODIES) {
      const bodyId = nameId(mujoco.mjtObj.mjOBJ_BODY.value, name);
      this.propBodies[name] = bodyId;
      const jntAdr = model.body_jntadr[bodyId];
      this.propJoints[name] = {
        qposAdr: model.jnt_qposadr[jntAdr],
        dofAdr: model.jnt_dofadr[jntAdr],
      };
    }
    // The robot itself (however many DOF it has — this changed once when
    // hands were added) occupies every qpos/qvel index before the first
    // prop body, since <include file="g1_with_hands.xml"/> is the first
    // thing in world.xml. Derived rather than hardcoded so this doesn't
    // silently break again if the model changes. Used by _resetAfterFall
    // to reset only the robot, not the whole world.
    this.robotNq = this.propJoints[GRABBABLE_BODIES[0]].qposAdr;
    this.robotNv = this.propJoints[GRABBABLE_BODIES[0]].dofAdr;
    this.robotQpos0 = Float32Array.from(model.qpos0.slice(0, this.robotNq));
    this.zoneBodies = {};
    for (const zoneName of Object.values(TASK_ZONES)) {
      this.zoneBodies[zoneName] = nameId(mujoco.mjtObj.mjOBJ_BODY.value, zoneName);
    }
    this.delivered = {};
    for (const name of GRABBABLE_BODIES) this.delivered[name] = false;
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

  /** Q = grab/release with right hand, E = grab/release with left hand. */
  _updateGrab(data) {
    const qDown = !!this.keys["KeyQ"];
    const eDown = !!this.keys["KeyE"];
    const qEdge = qDown && !this._prevKeys["KeyQ"];
    const eEdge = eDown && !this._prevKeys["KeyE"];
    this._prevKeys["KeyQ"] = qDown;
    this._prevKeys["KeyE"] = eDown;

    if (qEdge) { console.log("[grab] Q pressed"); this._toggleGrab(data, "right"); }
    if (eEdge) { console.log("[grab] E pressed"); this._toggleGrab(data, "left"); }
  }

  _bodyPos(data, bodyId) {
    return [data.xpos[bodyId * 3], data.xpos[bodyId * 3 + 1], data.xpos[bodyId * 3 + 2]];
  }

  _dist(a, b) {
    return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  }

  _toggleGrab(data, hand) {
    const held = hand === "right" ? this.heldByRightHand : this.heldByLeftHand;
    if (held) {
      console.log(`[grab] ${hand} hand released ${held}`);
      if (hand === "right") this.heldByRightHand = null; else this.heldByLeftHand = null;
      this._updateStatusText();
      return;
    }
    const wristBody = hand === "right" ? this.rightWristBody : this.leftWristBody;
    const wristPos = this._bodyPos(data, wristBody);
    let nearest = null, nearestDist = GRAB_RANGE;
    let closestOverall = Infinity, closestName = null;
    for (const name of GRABBABLE_BODIES) {
      // don't let both hands grab the same prop
      if (this.heldByRightHand === name || this.heldByLeftHand === name) continue;
      const propPos = this._bodyPos(data, this.propBodies[name]);
      const d = this._dist(wristPos, propPos);
      if (d < closestOverall) { closestOverall = d; closestName = name; }
      if (d < nearestDist) { nearest = name; nearestDist = d; }
    }
    if (nearest) {
      console.log(`[grab] ${hand} hand grabbed ${nearest} (${nearestDist.toFixed(2)}m)`);
      if (hand === "right") this.heldByRightHand = nearest; else this.heldByLeftHand = nearest;
    } else {
      console.log(`[grab] ${hand} hand: nothing in reach (closest is ${closestName} at ${closestOverall.toFixed(2)}m, need < ${GRAB_RANGE}m)`);
    }
    this._updateStatusText();
  }

  /** A prop counts as delivered once it's been within its zone (and not
   * being carried) at least once — sticky, so a slight roll/nudge
   * afterward (the ball especially) doesn't un-deliver it. Picking it back
   * up and carrying it away does clear it, though, so it can be redone. */
  _updateDelivery(data) {
    for (const propName of GRABBABLE_BODIES) {
      if (this.heldByRightHand === propName || this.heldByLeftHand === propName) {
        this.delivered[propName] = false;
        continue;
      }
      if (this.delivered[propName]) continue; // already sticky-delivered
      const zoneName = TASK_ZONES[propName];
      const propPos = this._bodyPos(data, this.propBodies[propName]);
      const zonePos = this._bodyPos(data, this.zoneBodies[zoneName]);
      const dxy = Math.hypot(propPos[0] - zonePos[0], propPos[1] - zonePos[1]);
      if (dxy < ZONE_RADIUS) {
        this.delivered[propName] = true;
        console.log(`[task] ${propName} delivered!`);
      }
    }
  }

  _updateStatusText() {
    const el = document.getElementById("grab-status");
    if (!el) return;
    const hands = `Right hand: ${this.heldByRightHand || "empty"} | Left hand: ${this.heldByLeftHand || "empty"}`;
    const doneCount = Object.values(this.delivered).filter(Boolean).length;
    const total = GRABBABLE_BODIES.length;
    const task = doneCount === total
      ? "Dishwasher loaded! All plates in."
      : `Dishwasher: ${doneCount}/${total} plates loaded`;
    el.innerHTML = `${hands}<br>${task}`;
  }

  /** Snap any held prop's position to its holding wrist every physics
   * substep, and zero its velocity so it doesn't fight the teleport. */
  _followHeldProps(data) {
    if (this.heldByRightHand) this._snapPropToHand(data, this.heldByRightHand, this.rightWristBody);
    if (this.heldByLeftHand) this._snapPropToHand(data, this.heldByLeftHand, this.leftWristBody);
  }

  _snapPropToHand(data, propName, wristBody) {
    const { qposAdr, dofAdr } = this.propJoints[propName];
    data.qpos[qposAdr + 0] = data.xpos[wristBody * 3 + 0];
    data.qpos[qposAdr + 1] = data.xpos[wristBody * 3 + 1];
    data.qpos[qposAdr + 2] = data.xpos[wristBody * 3 + 2] - 0.05;
    for (let i = 0; i < 6; i++) data.qvel[dofAdr + i] = 0;
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
    console.log("[reset] robot fell over — auto-resetting");
    // Reset only the robot's own qpos/qvel (the first robotNq/robotNv
    // entries), not the whole world via mj_resetData: falling shouldn't
    // wipe already-delivered plates or send everything flying back to its
    // start position — just the robot teleports back to spawn. Whatever it
    // was holding comes back with it (heldBy*Hand isn't cleared), since
    // _followHeldProps will snap it to the reset wrist on the next step.
    data.qpos.set(this.robotQpos0, 0);
    for (let i = 0; i < this.robotNv; i++) data.qvel[i] = 0;
    this.mujoco.mj_forward(this.model, data);

    this.h.fill(0);
    this.c.fill(0);
    this.prevAction.fill(0);
    this.targetDofPos = Float32Array.from(CFG.legDefaultAngles);
    this.stepCounter = 0;
    this.cmd = [0, 0, 0];
    this.jumpState = "idle";
    this.jumpTimer = 0;
    this.jumpOverride = null;
    this._updateStatusText();
  }

  /** Call once per physics substep (mj_step), before stepping. */
  beforeStep(data, dt) {
    this._checkFall(data, dt);
    this.stepCounter++;
    if (this.stepCounter % CFG.controlDecimation === 0) {
      this._updateCommand();
      this._runPolicy(data); // fire-and-forget; ~sub-ms on this tiny net
    }
    this._updateJump(dt);
    this._followHeldProps(data);

    const legTarget = this.jumpOverride || this.targetDofPos;
    for (let i = 0; i < 12; i++) {
      const q = data.qpos[7 + i];
      const dq = data.qvel[6 + i];
      data.ctrl[i] = CFG.kpsLeg[i] * (legTarget[i] - q) - CFG.kdsLeg[i] * dq;
    }
    // Actuator layout: waist+left-arm 12-21, left hand 22-28, right arm
    // 29-35, right hand 36-42 (see CFG comment above).
    for (let i = 0; i < CFG.waistLeftArmAngles.length; i++) {
      data.ctrl[12 + i] = CFG.waistLeftArmAngles[i];
    }
    const leftHandTarget = this.heldByLeftHand ? CFG.leftHandClosed : CFG.leftHandOpen;
    for (let i = 0; i < leftHandTarget.length; i++) {
      data.ctrl[22 + i] = leftHandTarget[i];
    }
    for (let i = 0; i < CFG.rightArmAngles.length; i++) {
      data.ctrl[29 + i] = CFG.rightArmAngles[i];
    }
    const rightHandTarget = this.heldByRightHand ? CFG.rightHandClosed : CFG.rightHandOpen;
    for (let i = 0; i < rightHandTarget.length; i++) {
      data.ctrl[36 + i] = rightHandTarget[i];
    }
  }

  /** Call once per rendered frame (not per physics substep). */
  update(data) {
    this._updateGrab(data);
    this._updateDelivery(data);
    this._updateStatusText();
  }
}
