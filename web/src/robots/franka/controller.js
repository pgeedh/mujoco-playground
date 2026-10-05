// Bimanual Franka Panda biohazard lab ("Containment Breach"). Two Panda arms are driven by
// damped-least-squares inverse kinematics (MuJoCo site Jacobians) toward Cartesian targets you move
// with the keyboard; the hands grasp with real contact friction.
// Mission: a sample rack at the far left is reachable only by the LEFT arm, the biohazard case at the
// far right only by the RIGHT arm. Relay each tube through the airlock tray in the middle, drop all
// four into the case, then press SEAL. A spilled tube (off the bench) contaminates the room.
import * as THREE from "three";
import { makeRng } from "../../core/bench.js";

const ARMS = ["l_", "r_"];
const MOVE_SPEED = 0.3;      // m/s of the Cartesian target
const YAW_SPEED = 1.5;       // rad/s
const MAX_LEAD = 0.006;      // target may lead the gripper by this per IK step (m): ~0.6 m/s free motion
const DQ_MAX = 0.1;          // rad per control step
const HELD_LEAD = 0.0025, HELD_DQ_MAX = 0.04;   // ~0.25 m/s while the gripper is closed, so cubes don't slip out
const LAMBDA = 0.05;         // DLS damping
const CTRL_EVERY = 5;        // physics steps per IK update
const TABLE_Z = 0.4;
const REACH = { l_: { x: [-0.82, 0.25] }, r_: { x: [-0.25, 0.82] } };
const CASE = { x: 0.60, y: 0.16, hx: 0.105, hy: 0.068, top: 0.47 };   // biohazard case opening
const N_TUBES = 4;
const MOVE_KEYS = ["KeyW", "KeyA", "KeyS", "KeyD", "KeyR", "KeyF", "KeyQ", "KeyE", "Space", "Tab"];

function solve(A, b, n) {            // Gaussian elimination, A is n*n row-major
  const M = A.slice(), x = b.slice();
  for (let i = 0; i < n; i++) {
    let p = i;
    for (let r = i + 1; r < n; r++) if (Math.abs(M[r * n + i]) > Math.abs(M[p * n + i])) p = r;
    if (p !== i) {
      for (let c = 0; c < n; c++) { const t = M[i * n + c]; M[i * n + c] = M[p * n + c]; M[p * n + c] = t; }
      const t = x[i]; x[i] = x[p]; x[p] = t;
    }
    const d = M[i * n + i] || 1e-9;
    for (let r = i + 1; r < n; r++) {
      const f = M[r * n + i] / d;
      for (let c = i; c < n; c++) M[r * n + c] -= f * M[i * n + c];
      x[r] -= f * x[i];
    }
  }
  for (let i = n - 1; i >= 0; i--) {
    let s = x[i];
    for (let c = i + 1; c < n; c++) s -= M[i * n + c] * x[c];
    x[i] = s / (M[i * n + i] || 1e-9);
  }
  return x;
}

export class FrankaController {
  constructor() {
    this.ready = false;
    this.keys = {}; this._prev = {};
    this.anyInput = false;
    window.addEventListener("keydown", (e) => {
      if (MOVE_KEYS.includes(e.code)) { this.keys[e.code] = true; this.anyInput = true; if (e.code === "Tab" || e.code === "Space") e.preventDefault(); }
    });
    window.addEventListener("keyup", (e) => { this.keys[e.code] = false; });
  }

  bindModel(model, mujoco, ctx) {
    this.model = model; this.mujoco = mujoco;
    const id = (type, name) => mujoco.mj_name2id(model, type, name);
    const T = mujoco.mjtObj;
    this.arms = ARMS.map((p) => {
      const joints = [...Array(7)].map((_, i) => id(T.mjOBJ_JOINT.value, `${p}joint${i + 1}`));
      return {
        p,
        sid: id(T.mjOBJ_SITE.value, `${p}grip`),
        qadr: joints.map((j) => model.jnt_qposadr[j]),
        dadr: joints.map((j) => model.jnt_dofadr[j]),
        lo: joints.map((j) => model.jnt_range[j * 2]),
        hi: joints.map((j) => model.jnt_range[j * 2 + 1]),
        act0: id(T.mjOBJ_ACTUATOR.value, `${p}actuator1`),
        target: [0, 0, 0], yaw: 0, R0: new Array(9).fill(0), closed: false, qcmd: new Array(7).fill(0),
      };
    });
    this.tubes = [...Array(N_TUBES)].map((_, i) => {
      const body = id(T.mjOBJ_BODY.value, `tube${i}`);
      const j = model.body_jntadr[body];
      return { name: `tube${i}`, body, qadr: model.jnt_qposadr[j], dadr: model.jnt_dofadr[j] };
    });
    this.sealQ = model.jnt_qposadr[id(T.mjOBJ_JOINT.value, "seal_slide")];
    this.ctx = ctx || null;
    // Safe defaults until the bench calls reset(): the HUD is drawn every frame, including before that.
    this.secured = new Set(); this.fallen = new Set(); this.spills = 0; this.sealed = false; this.active = 0;
    this._attachAlarm();
    // The WASM bindings write outputs into their own heap buffers, not plain typed arrays.
    this.jacpBuf = new mujoco.DoubleBuffer(3 * model.nv);
    this.jacrBuf = new mujoco.DoubleBuffer(3 * model.nv);
  }

  async load() { this.ready = true; }
  hasInput() { return this.anyInput; }

  reset(data, seed) {
    const m = this.model;
    data.qpos.set(m.key_qpos.subarray(0, m.nq));
    data.qvel.fill(0);
    data.ctrl.set(m.key_ctrl.subarray(0, m.nu));
    const rng = makeRng(seed * 7919 + 13);
    const u = (a, b) => a + (b - a) * rng();
    // Seed shuffles which rack slot each tube sits in and nudges them a little along their rails.
    const slots = [...Array(N_TUBES).keys()].sort(() => rng() - 0.5);
    this.tubes.forEach((t, i) => {
      for (let k = 0; k < 7; k++) data.qpos[t.qadr + k] = m.key_qpos[this.tubes[slots[i]].qadr + k];
      data.qpos[t.qadr] += u(-0.012, 0.012);
    });
    this.mujoco.mj_forward(m, data);
    this.arms.forEach((a) => {
      for (let k = 0; k < 3; k++) a.target[k] = data.site_xpos[a.sid * 3 + k];
      for (let k = 0; k < 9; k++) a.R0[k] = data.site_xmat[a.sid * 9 + k];
      a.yaw = 0; a.closed = false;
      for (let c = 0; c < 7; c++) a.qcmd[c] = data.qpos[a.qadr[c]];
    });
    this.active = 0; this.step = 0; this.anyInput = false;
    this.spills = 0; this.fallen = new Set(); this.secured = new Set();
    this.sealed = false; this.pressT = 0; this.simTime = 0; this.transfers = 0;
    this._updateStatusText();
  }

  _pos(data, body) { return [data.xpos[body * 3], data.xpos[body * 3 + 1], data.xpos[body * 3 + 2]]; }

  _edge(code) { const d = !!this.keys[code], e = d && !this._prev[code]; this._prev[code] = d; return e; }

  _control(data, dt) {
    if (this._edge("Tab")) { this.active = 1 - this.active; }
    const arm = this.arms[this.active];
    if (this._edge("Space")) arm.closed = !arm.closed;
    const k = this.keys, v = MOVE_SPEED * dt;
    // Keys are camera-relative: W/S = away from / toward the operator (+y/-y), A/D = left/right.
    if (k.KeyW) arm.target[1] += v;
    if (k.KeyS) arm.target[1] -= v;
    if (k.KeyA) arm.target[0] -= v;
    if (k.KeyD) arm.target[0] += v;
    if (k.KeyR) arm.target[2] += v;
    if (k.KeyF) arm.target[2] -= v;
    if (k.KeyQ) arm.yaw += YAW_SPEED * dt;
    if (k.KeyE) arm.yaw -= YAW_SPEED * dt;
    const [xl, xh] = REACH[arm.p].x;
    arm.target[0] = Math.min(xh, Math.max(xl, arm.target[0]));
    arm.target[1] = Math.min(0.45, Math.max(-0.3, arm.target[1]));
    arm.target[2] = Math.min(1.0, Math.max(TABLE_Z + 0.002, arm.target[2]));

    for (const a of this.arms) this._ik(data, a);
  }

  _ik(data, a) {
    const m = this.model;
    const site = [data.site_xpos[a.sid * 3], data.site_xpos[a.sid * 3 + 1], data.site_xpos[a.sid * 3 + 2]];
    // Don't let the target run away from a gripper that's blocked (e.g. pressing on the table).
    let ep = [a.target[0] - site[0], a.target[1] - site[1], a.target[2] - site[2]];
    const n = Math.hypot(ep[0], ep[1], ep[2]);
    const lead = a.closed ? HELD_LEAD : MAX_LEAD;
    if (n > lead) {
      const s = lead / n;
      for (let i = 0; i < 3; i++) a.target[i] = site[i] + ep[i] * s;
      ep = ep.map((x) => x * s);
    }
    this.mujoco.mj_jacSite(m, data, this.jacpBuf, this.jacrBuf, a.sid);
    const jacp = this.jacpBuf.GetView(), jacr = this.jacrBuf.GetView();
    const J = new Array(42);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 7; c++) {
      J[r * 7 + c] = jacp[r * m.nv + a.dadr[c]];
      J[(r + 3) * 7 + c] = jacr[r * m.nv + a.dadr[c]];
    }
    // Rotation error: target orientation = Rz(yaw) * R0.
    const R = [...Array(9)].map((_, i) => data.site_xmat[a.sid * 9 + i]);
    const cy = Math.cos(a.yaw), sy = Math.sin(a.yaw);
    const Rz = [cy, -sy, 0, sy, cy, 0, 0, 0, 1];
    const Rt = new Array(9).fill(0);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) Rt[i * 3 + j] += Rz[i * 3 + k] * a.R0[k * 3 + j];
    const Re = new Array(9).fill(0);   // Rt * R^T
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) Re[i * 3 + j] += Rt[i * 3 + k] * R[j * 3 + k];
    const er = [0.5 * (Re[7] - Re[5]), 0.5 * (Re[2] - Re[6]), 0.5 * (Re[3] - Re[1])];
    // Orientation is held with a lower weight than position (WR), so tiny position steps are not
    // swamped by orientation corrections (that made the arm wander) yet the hand stays upright.
    const WR = 0.5, rn = Math.hypot(er[0], er[1], er[2]) * 2, rs = rn > 0.05 ? 0.05 / rn : 1;
    for (let c = 0; c < 7; c++) for (let r = 3; r < 6; r++) J[r * 7 + c] *= WR;
    const e = [...ep, WR * er[0] * 2 * rs, WR * er[1] * 2 * rs, WR * er[2] * 2 * rs];
    // dq = J^T (J J^T + lambda^2 I)^-1 e
    const A = new Array(36).fill(0);
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) {
      let s = 0;
      for (let c = 0; c < 7; c++) s += J[i * 7 + c] * J[j * 7 + c];
      A[i * 6 + j] = s + (i === j ? LAMBDA * LAMBDA : 0);
    }
    const y = solve(A, e, 6);
    const dqs = new Array(7);
    let biggest = 0;
    for (let c = 0; c < 7; c++) {
      let dq = 0;
      for (let r = 0; r < 6; r++) dq += J[r * 7 + c] * y[r];
      dqs[c] = dq; biggest = Math.max(biggest, Math.abs(dq));
    }
    // Scale the whole step (not each joint) so the Cartesian direction is preserved.
    const lim = a.closed ? HELD_DQ_MAX : DQ_MAX, scale = biggest > lim ? lim / biggest : 1;
    for (let c = 0; c < 7; c++) {
      // Integrate the joint command (so gravity sag can't swallow small steps), but
      // never let it run more than 0.12 rad ahead of the real joint.
      const q = data.qpos[a.qadr[c]];
      a.qcmd[c] = Math.max(q - 0.12, Math.min(q + 0.12, a.qcmd[c] + dqs[c] * scale));
      data.ctrl[a.act0 + c] = Math.max(a.lo[c], Math.min(a.hi[c], a.qcmd[c]));
    }
    data.ctrl[a.act0 + 7] = a.closed ? 0 : 255;
  }

  beforeStep(data, dt) {
    this.simTime += dt;
    if (this.step++ % CTRL_EVERY === 0) this._control(data, dt * CTRL_EVERY);
    this._track(data, dt);
  }

  /** Spills, tubes secured in the case, and the SEAL button. */
  _track(data, dt) {
    for (const t of this.tubes) {
      const x = data.xpos[t.body * 3], y = data.xpos[t.body * 3 + 1], z = data.xpos[t.body * 3 + 2];
      if (z < TABLE_Z - 0.1) {
        if (!this.fallen.has(t.name)) { this.fallen.add(t.name); this.spills++; }
        this.secured.delete(t.name); continue;
      }
      const speed = Math.hypot(data.qvel[t.dadr], data.qvel[t.dadr + 1], data.qvel[t.dadr + 2]);
      const inside = Math.abs(x - CASE.x) < CASE.hx && Math.abs(y - CASE.y) < CASE.hy && z < CASE.top && speed < 0.1;
      if (inside) this.secured.add(t.name); else if (!(Math.abs(x - CASE.x) < CASE.hx && Math.abs(y - CASE.y) < CASE.hy)) this.secured.delete(t.name);
    }
    const q = data.qpos[this.sealQ];
    this.pressT = q < -0.012 ? this.pressT + dt : 0;
    if (this.pressT > 0.25 && this.secured.size === N_TUBES) this.sealed = true;
  }

  _attachAlarm() {
    const scene = this.ctx && this.ctx.scene;
    if (!scene) return;
    if (!this.alarm) { this.alarm = new THREE.PointLight(0xff2a1a, 0, 6, 1.4); this.alarm.position.set(-1.0, 1.15, -0.55); }
    scene.add(this.alarm);
  }
  dispose() { if (this.alarm && this.alarm.parent) this.alarm.parent.remove(this.alarm); }

  update(data) {
    if (this.alarm) this.alarm.intensity = 2 + 5 * Math.max(0, Math.sin(performance.now() / 260));     // rotating beacon pulse
    this._updateStatusText();
  }

  _updateStatusText() {
    const el = document.getElementById("grab-status");
    if (!el || !this.arms) return;
    const g = (a) => (a.closed ? "closed" : "open");
    const next = this.secured.size < N_TUBES ? `Tubes secured ${this.secured.size}/${N_TUBES}. Relay each one through the AIRLOCK (middle tray) to the case.` : (this.sealed ? "Case sealed. Contained." : "All tubes secured: press the red SEAL button with a closed gripper.");
    el.innerHTML = `<b>${next}</b><br>Active arm: <b>${this.active === 0 ? "LEFT" : "RIGHT"}</b> (Tab to switch) · grippers: left ${g(this.arms[0])} | right ${g(this.arms[1])}<br>Left arm reaches the rack only · right arm reaches the case only`;
  }

  isSuccess() { return this.sealed && this.secured.size === N_TUBES; }
  getMetrics() { return { "tubes secured": `${this.secured.size}/${N_TUBES}`, spills: this.spills, sealed: this.sealed ? "yes" : "no" }; }
  score(time, limit, success) {
    let s = 17 * this.secured.size + (this.sealed ? 22 : 0);
    if (success) s += 10 * Math.max(0, 1 - time / limit);
    return Math.max(0, Math.min(100, s - 10 * this.spills));
  }
}
