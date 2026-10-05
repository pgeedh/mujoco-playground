// Mars sample-return mission driven with a Clearpath Husky (4-wheel skid steer).
// The rover localises itself with SLAM (front lidar + wheel odometry + scan matching, see slam.js),
// so the map you navigate by is the one it builds, and its position estimate can drift.
// Mission: reach the survey beacon, collect the sample at the marked site, return to the lander.
import * as THREE from "three";
import { makeRng } from "../../core/bench.js";
import { Minimap } from "./minimap.js";
import { SlamSystem, makeWorld } from "./slam.js";

const WHEEL_R = 0.1651, TRACK = 0.555, BASE_Z = 0.1323;
const V_MAX = 1.0, W_MAX = 1.4, ACCEL = 0.9, ALPHA = 2.5;            // Husky tops out around 1 m/s
const REACH = 3.2, HOME_REACH = 4.0, SAMPLE_REACH = 3.0, COLLECT_TIME = 3.0;
const P_IDLE = 25, E_CAP = 42000, MOTOR_EFF = 0.7;                     // W, J (mission battery pack), -
const CONTROL_KEYS = ["KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "Space", "KeyE"];
const DUST_N = 260;

export class MarsController {
  constructor() {
    this.ready = false; this.keys = {}; this.anyInput = false;
    this.map = new Minimap();
    window.addEventListener("keydown", (e) => { if (CONTROL_KEYS.includes(e.code)) { this.keys[e.code] = true; if (e.code !== "KeyE") this.anyInput = true; } });
    window.addEventListener("keyup", (e) => { this.keys[e.code] = false; });
  }

  bindModel(model, mujoco, ctx) {
    this.model = model; this.mujoco = mujoco; this.ctx = ctx;
    const T = mujoco.mjtObj, id = (t, n) => mujoco.mj_name2id(model, t, n);
    this.body = id(T.mjOBJ_BODY.value, "husky");
    this.jadr = model.jnt_qposadr[model.body_jntadr[this.body]];
    this.dadr = model.jnt_dofadr[model.body_jntadr[this.body]];
    this.acts = ["a_fl", "a_fr", "a_rl", "a_rr"].map((n) => id(T.mjOBJ_ACTUATOR.value, n));
    this.wheelDof = ["wj_fl", "wj_fr", "wj_rl", "wj_rr"].map((n) => model.jnt_dofadr[id(T.mjOBJ_JOINT.value, n)]);
    const mocap = (n) => model.body_mocapid[id(T.mjOBJ_BODY.value, n)];
    this.mc = { alpha: mocap("beacon_alpha"), sample: mocap("beacon_sample"), home: mocap("beacon_home"), item: mocap("sample") };
    this._attachVisuals();
  }

  async load() {
    this.mission = await (await fetch("./assets/scenes/rover_mars/mission.json")).json();
    this.heights = await this._loadHeights("./assets/scenes/rover_mars/" + this.mission.heightmap, this.mission.zmax);
    const lz = this._ground(this.mission.lander[0], this.mission.lander[1]);
    this.world = makeWorld((x, y) => this._ground(x, y), this.mission.rocks, this.mission.half_extent,
      [{ x: this.mission.lander[0], y: this.mission.lander[1], hx: 1.15, hy: 1.15, z0: lz + 0.5, z1: lz + 2.0 }]);
    this.ready = true;
  }

  async _loadHeights(url, zmax) {
    const bmp = await createImageBitmap(await (await fetch(url)).blob(), { colorSpaceConversion: "none" });
    const c = document.createElement("canvas"); c.width = bmp.width; c.height = bmp.height;
    const g = c.getContext("2d"); g.drawImage(bmp, 0, 0);
    const d = g.getImageData(0, 0, bmp.width, bmp.height).data, n = bmp.width, h = [];
    for (let i = 0; i < n; i++) { const row = new Float32Array(n), src = n - 1 - i; for (let j = 0; j < n; j++) row[j] = (d[(src * n + j) * 4] / 255) * zmax; h.push(row); }
    return h;
  }

  hasInput() { return this.anyInput; }

  _ground(x, y) {            // bilinear terrain height (same data MuJoCo collides with)
    const heights = this.heights, R = this.mission.half_extent, n = heights.length;
    const fx = Math.min(n - 1.001, Math.max(0, (x + R) / (2 * R) * (n - 1)));
    const fy = Math.min(n - 1.001, Math.max(0, (y + R) / (2 * R) * (n - 1)));
    const j = Math.floor(fx), i = Math.floor(fy), tx = fx - j, ty = fy - i;
    return heights[i][j] * (1 - tx) * (1 - ty) + heights[i][j + 1] * tx * (1 - ty) + heights[i + 1][j] * (1 - tx) * ty + heights[i + 1][j + 1] * tx * ty;
  }

  // ---- visuals: lidar returns and wheel dust -----------------------------------
  _attachVisuals() {
    const scene = this.ctx && this.ctx.scene;
    if (!scene) return;
    if (!this.lidarPts) {
      const lg = new THREE.BufferGeometry(); lg.setAttribute("position", new THREE.BufferAttribute(new Float32Array(270 * 3), 3)); lg.setDrawRange(0, 0);
      this.lidarPts = new THREE.Points(lg, new THREE.PointsMaterial({ color: 0x7cf2e6, size: 0.11, sizeAttenuation: true, transparent: true, opacity: 0.9, depthTest: false }));
      this.lidarPts.frustumCulled = false; this.lidarPts.renderOrder = 5;
      const dg = new THREE.BufferGeometry(); dg.setAttribute("position", new THREE.BufferAttribute(new Float32Array(DUST_N * 3), 3));
      this.dust = new THREE.Points(dg, new THREE.PointsMaterial({ color: 0xd9a77a, size: 0.16, sizeAttenuation: true, transparent: true, opacity: 0.5, depthWrite: false }));
      this.dust.frustumCulled = false;
      this.dustVel = new Float32Array(DUST_N * 3); this.dustAge = new Float32Array(DUST_N).fill(99); this.dustNext = 0;
    }
    scene.add(this.lidarPts); scene.add(this.dust);
  }
  dispose() {
    this.map.hide();
    if (this.lidarPts && this.lidarPts.parent) this.lidarPts.parent.remove(this.lidarPts);
    if (this.dust && this.dust.parent) this.dust.parent.remove(this.dust);
  }

  reset(data, seed) {
    const m = this.mission, rng = makeRng(seed * 104729 + 7);
    this.sampleIdx = ((seed % 4) + 4) % 4;
    this.sample = m.samples[this.sampleIdx];
    this.home = [m.lander[0] + 4, m.lander[1]];
    const place = (mc, x, y, dz) => { data.mocap_pos[mc * 3] = x; data.mocap_pos[mc * 3 + 1] = y; data.mocap_pos[mc * 3 + 2] = this._ground(x, y) + dz; };
    place(this.mc.alpha, m.alpha[0], m.alpha[1], 0);
    place(this.mc.sample, this.sample[0], this.sample[1], 0);
    place(this.mc.item, this.sample[0], this.sample[1], 0.2);
    place(this.mc.home, this.home[0], this.home[1], 0);
    const yaw = Math.atan2(m.alpha[1] - m.start[1], m.alpha[0] - m.start[0]) + (rng() - 0.5) * 0.4;
    this._setPose(data, m.start[0], m.start[1], yaw);
    this.v = 0; this.w = 0; this.step = 0; this.anyInput = false;
    this.phase = 0; this.collect = 0; this.hasSample = false;
    this.bumps = 0; this._bumping = false; this.rollovers = 0; this._flipT = 0; this.dist = 0; this._last = null;
    this.energy = E_CAP; this.dead = false; this.slip = 0; this.pitch = 0; this.roll = 0; this.speed = 0;
    this.mujoco.mj_forward(this.model, data);
    const p = this._pose(data);
    this.slam = new SlamSystem({ world: this.world, half: m.half_extent, seed: seed + 1 });
    this.slam.reset({ x: p.x, y: p.y, yaw: p.yaw, z: p.z });
    this.trailEst = []; this.trailOdo = []; this.trailTruth = []; this._trailT = 0;
    if (this.dustAge) this.dustAge.fill(99);
    this.map.show(this.mission, this.heights);
  }

  _setPose(data, x, y, yaw) {
    const a = this.jadr;
    data.qpos[a] = x; data.qpos[a + 1] = y; data.qpos[a + 2] = this._ground(x, y) + BASE_Z + 0.2;
    data.qpos[a + 3] = Math.cos(yaw / 2); data.qpos[a + 4] = 0; data.qpos[a + 5] = 0; data.qpos[a + 6] = Math.sin(yaw / 2);
    for (let i = 0; i < 6; i++) data.qvel[this.dadr + i] = 0;
  }

  _pose(data) {
    const b = this.body, x = data.xpos[b * 3], y = data.xpos[b * 3 + 1];
    return { x, y, z: data.xpos[b * 3 + 2], yaw: Math.atan2(data.xmat[b * 9 + 3], data.xmat[b * 9]), up: data.xmat[b * 9 + 8] };
  }

  beforeStep(data, dt) {
    const k = this.keys;
    const fwd = (k.KeyW || k.ArrowUp ? 1 : 0) - (k.KeyS || k.ArrowDown ? 1 : 0);
    const turn = (k.KeyA || k.ArrowLeft ? 1 : 0) - (k.KeyD || k.ArrowRight ? 1 : 0);
    const vT = this.dead || k.Space ? 0 : fwd * V_MAX, wT = this.dead ? 0 : turn * W_MAX;
    const slew = (cur, tgt, rate) => cur + Math.max(-rate * dt, Math.min(rate * dt, tgt - cur));
    this.v = slew(this.v, vT, ACCEL * (vT === 0 ? 2 : 1)); this.w = slew(this.w, wT, ALPHA);
    const wl = (this.v - this.w * TRACK / 2) / WHEEL_R, wr = (this.v + this.w * TRACK / 2) / WHEEL_R;
    data.ctrl[this.acts[0]] = data.ctrl[this.acts[2]] = wl;
    data.ctrl[this.acts[1]] = data.ctrl[this.acts[3]] = wr;

    if (this.step++ % 10 !== 0) return;
    const tick = dt * 10, p = this._pose(data);
    if (this._last) this.dist += Math.hypot(p.x - this._last[0], p.y - this._last[1]);
    this._last = [p.x, p.y];
    // Telemetry from the physics state.
    const vw = (i) => data.qvel[this.wheelDof[i]];
    const wLeft = (vw(0) + vw(2)) / 2, wRight = (vw(1) + vw(3)) / 2;
    const b = this.body, cy = Math.cos(p.yaw), sy = Math.sin(p.yaw);
    const vFwd = data.qvel[this.dadr] * cy + data.qvel[this.dadr + 1] * sy;
    const vWheel = (WHEEL_R * (wLeft + wRight)) / 2;
    this.speed = Math.abs(vFwd);
    this.slip = Math.abs(vWheel) > 0.25 ? Math.max(0, Math.min(1, 1 - Math.abs(vFwd) / Math.abs(vWheel))) : 0;
    this.pitch = Math.asin(Math.max(-1, Math.min(1, data.xmat[b * 9 + 6]))); this.roll = Math.asin(Math.max(-1, Math.min(1, data.xmat[b * 9 + 7])));
    // Battery: idle electronics + motor power (torque x speed through the motor controller).
    let mech = 0;
    for (let i = 0; i < 4; i++) { const tau = Math.max(-400, Math.min(400, 150 * (data.ctrl[this.acts[i]] - vw(i)))); mech += Math.abs(tau * vw(i)); }
    this.energy = Math.max(0, this.energy - (P_IDLE + mech / MOTOR_EFF) * tick);
    if (this.energy <= 0) this.dead = true;
    // SLAM: wheel odometry from the encoders, lidar scans from the true pose.
    const gx = data.qvel[this.dadr + 3], gy = data.qvel[this.dadr + 4], gz = data.qvel[this.dadr + 5];    // body-frame gyro
    const yawRate = data.xmat[b * 9 + 6] * gx + data.xmat[b * 9 + 7] * gy + data.xmat[b * 9 + 8] * gz;
    this.slam.update(tick, { x: p.x, y: p.y, z: p.z, yaw: p.yaw }, wLeft, wRight, { v: vFwd, w: yawRate });
    // Collisions with boulders (rising edge of the bump sensor).
    const hit = data.sensordata[0] > 40;
    if (hit && !this._bumping) this.bumps++;
    this._bumping = hit;
    // Rollover: upside down or on its side for 2 s -> set upright where it is.
    this._flipT = p.up < 0.3 ? this._flipT + tick : 0;
    if (this._flipT > 2) { this.rollovers++; this._flipT = 0; this._setPose(data, p.x, p.y, p.yaw); }
    this._missionStep(data, p, tick);
    this._trailT += tick;
    if (this._trailT > 0.5) {
      this._trailT = 0;
      this.trailEst.push([this.slam.est.x, this.slam.est.y]); this.trailOdo.push([this.slam.odo.x, this.slam.odo.y]); this.trailTruth.push([p.x, p.y]);
      for (const t of [this.trailEst, this.trailOdo, this.trailTruth]) if (t.length > 500) t.shift();
    }
  }

  _dist(p, t) { return Math.hypot(p.x - t[0], p.y - t[1]); }

  _missionStep(data, p, tick) {
    const m = this.mission;
    if (this.phase === 0 && this._dist(p, m.alpha) < REACH) this.phase = 1;
    else if (this.phase === 1) {
      const near = this._dist(p, this.sample) < SAMPLE_REACH && Math.abs(this.v) < 0.3;
      if (near && this.keys.KeyE) this.collect += tick; else if (!this.keys.KeyE) this.collect = Math.max(0, this.collect - tick);
      if (this.collect >= COLLECT_TIME) { this.phase = 2; this.hasSample = true; data.mocap_pos[this.mc.item * 3 + 2] = -50; }
    } else if (this.phase === 2 && this._dist(p, this.home) < HOME_REACH) this.phase = 3;
  }

  update(data) {
    if (!this.sample || !this.slam) return;
    const p = this._pose(data), est = this.slam.est;
    const tgt = this._goal(p);
    const where = (t) => Math.hypot(est.x - t[0], est.y - t[1]);
    const msg = this.phase === 0 ? "Drive to the BLUE survey beacon"
      : this.phase === 1 ? (this._dist(p, this.sample) < SAMPLE_REACH ? "At the sample site: stop and hold E to collect" : "Drive to the GREEN sample site")
        : this.phase === 2 ? "Sample secured! Return to the YELLOW beacon at the lander" : "Mission complete";
    const el = document.getElementById("grab-status");
    if (el) {
      const bar = this.phase === 1 && this.collect > 0 ? ` · collecting ${Math.min(100, Math.round(this.collect / COLLECT_TIME * 100))}%` : "";
      const batt = Math.round(this.energy / E_CAP * 100);
      el.innerHTML = `<b>${msg}</b>${bar}<br>Speed ${this.speed.toFixed(1)} m/s · Battery ${batt}% · Wheel slip ${Math.round(this.slip * 100)}% · Tilt ${Math.round(Math.max(Math.abs(this.pitch), Math.abs(this.roll)) * 57.3)}°`
        + `<br>Goal ${where(tgt).toFixed(0)} m (by SLAM) · position error: SLAM ${this.slam.errSlam.toFixed(1)} m vs odometry ${this.slam.errOdo.toFixed(1)} m`;
    }
    const marks = [
      { x: this.mission.lander[0], y: this.mission.lander[1], color: "#E8E8F0", shape: "square", label: "Lander" },
      { x: this.mission.alpha[0], y: this.mission.alpha[1], color: "#3399FF", label: "Survey", done: this.phase > 0 },
      { x: this.sample[0], y: this.sample[1], color: "#33FFAA", label: "Sample", done: this.hasSample },
      { x: this.home[0], y: this.home[1], color: "#FFD633", label: "Home", done: this.phase >= 3 },
    ];
    this.map.draw(marks, { grid: this.slam.grid, est, truth: p, trailEst: this.trailEst, trailOdo: this.trailOdo, trailTruth: this.trailTruth }, `Phase ${Math.min(this.phase + 1, 3)}/3`);
    this._updateVisuals(p);
  }

  _goal(p) { return this.phase === 0 ? this.mission.alpha : this.phase === 1 ? this.sample : this.phase === 2 ? this.home : [p.x, p.y]; }

  _updateVisuals(p) {
    if (!this.lidarPts) return;
    const hits = this.slam.lastHits, pos = this.lidarPts.geometry.attributes.position, zl = p.z + 0.05 + 0.02;
    const n = Math.min(hits.length, 270);
    for (let i = 0; i < n; i++) { pos.setXYZ(i, hits[i][0], zl, -hits[i][1]); }
    pos.needsUpdate = true; this.lidarPts.geometry.setDrawRange(0, n);
    // Dust kicked up by the wheels (slow fall in 0.38 g).
    const now = performance.now(), dt = Math.min(0.06, (now - (this._dustT || now)) / 1000); this._dustT = now;
    const dp = this.dust.geometry.attributes.position;
    if (this.speed > 0.25 || Math.abs(this.w) > 0.4) {
      const cy = Math.cos(p.yaw), sy = Math.sin(p.yaw), rate = Math.min(6, 2 + this.speed * 4 + this.slip * 10);
      for (let k = 0; k < rate; k++) {
        const i = this.dustNext++ % DUST_N, side = k % 2 ? 1 : -1, bx = p.x - cy * 0.3 - sy * 0.3 * side, by = p.y - sy * 0.3 + cy * 0.3 * side;
        dp.setXYZ(i, bx, this._ground(bx, by) + 0.08, -by);
        this.dustVel[i * 3] = -cy * 0.5 + (Math.random() - 0.5) * 0.5; this.dustVel[i * 3 + 1] = 0.5 + Math.random() * 0.6; this.dustVel[i * 3 + 2] = sy * 0.5 + (Math.random() - 0.5) * 0.5;
        this.dustAge[i] = 0;
      }
    }
    for (let i = 0; i < DUST_N; i++) {
      this.dustAge[i] += dt;
      if (this.dustAge[i] > 2.2) { dp.setY(i, -100); continue; }
      this.dustVel[i * 3 + 1] -= 3.71 * 0.35 * dt;
      dp.setXYZ(i, dp.getX(i) + this.dustVel[i * 3] * dt, Math.max(dp.getY(i) + this.dustVel[i * 3 + 1] * dt, -50), dp.getZ(i) + this.dustVel[i * 3 + 2] * dt);
    }
    dp.needsUpdate = true;
  }

  /** Chase camera hook used by main.js. */
  chaseTarget(data) { const p = this._pose(data); return { x: p.x, y: p.y, z: p.z, yaw: p.yaw }; }

  isSuccess() { return this.phase >= 3; }
  isFailed() { return this.dead; }
  failReason() { return "battery depleted"; }
  getMetrics() {
    return { "mission phase": `${Math.min(this.phase, 3)}/3`, battery: `${Math.round(this.energy / E_CAP * 100)}%`, "boulder hits": this.bumps, rollovers: this.rollovers,
      "SLAM error (m)": this.slam ? this.slam.rmsSlam.toFixed(2) : "0", "odometry error (m)": this.slam ? this.slam.rmsOdo.toFixed(2) : "0" };
  }
  score(time, limit, success) {
    let s = 30 * Math.min(this.phase, 3);
    if (success) s += 10 * Math.max(0, 1 - time / limit);
    return Math.max(0, Math.min(100, s - 2 * this.bumps - 5 * this.rollovers));
  }
}
