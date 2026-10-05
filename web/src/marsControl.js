// Mars sample-return mission driven with a Clearpath Husky (4-wheel skid steer).
// Mission: reach the survey beacon, collect the sample at the marked site, and
// return to the lander. The map shows relief, beacons and the rover's trail.
import { makeRng } from "./bench.js";
import { Minimap } from "./minimap.js";

const WHEEL_R = 0.1651, TRACK = 0.555, BASE_Z = 0.1323;
const V_MAX = 1.8, W_MAX = 2.0, ACCEL = 2.5, ALPHA = 4.0;
const REACH = 3.2, HOME_REACH = 4.0, SAMPLE_REACH = 3.0, COLLECT_TIME = 3.0;
const CONTROL_KEYS = ["KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "Space", "KeyE"];

export class MarsController {
  constructor() {
    this.ready = false; this.keys = {}; this.anyInput = false;
    this.map = new Minimap();
    window.addEventListener("keydown", (e) => {
      if (CONTROL_KEYS.includes(e.code)) { this.keys[e.code] = true; if (e.code !== "KeyE") this.anyInput = true; e.preventDefault && e.code.startsWith("Arrow") && e.preventDefault(); }
    });
    window.addEventListener("keyup", (e) => { this.keys[e.code] = false; });
  }

  bindModel(model, mujoco, ctx) {
    this.model = model; this.mujoco = mujoco; this.ctx = ctx;
    const T = mujoco.mjtObj, id = (t, n) => mujoco.mj_name2id(model, t, n);
    this.body = id(T.mjOBJ_BODY.value, "husky");
    this.jadr = model.jnt_qposadr[model.body_jntadr[this.body]];
    this.dadr = model.jnt_dofadr[model.body_jntadr[this.body]];
    this.acts = ["a_fl", "a_fr", "a_rl", "a_rr"].map((n) => id(T.mjOBJ_ACTUATOR.value, n));
    const mocap = (n) => model.body_mocapid[id(T.mjOBJ_BODY.value, n)];
    this.mc = { alpha: mocap("beacon_alpha"), sample: mocap("beacon_sample"), home: mocap("beacon_home"), item: mocap("sample") };
  }

  async load() {
    this.mission = await (await fetch("./assets/scenes/rover_mars/mission.json")).json();
    this.ready = true;
  }
  hasInput() { return this.anyInput; }

  _ground(x, y) {            // bilinear terrain height from the generated heightmap
    const { heights, half_extent: R } = this.mission, n = heights.length;
    const fx = Math.min(n - 1.001, Math.max(0, (x + R) / (2 * R) * (n - 1)));
    const fy = Math.min(n - 1.001, Math.max(0, (y + R) / (2 * R) * (n - 1)));
    const j = Math.floor(fx), i = Math.floor(fy), tx = fx - j, ty = fy - i;
    return heights[i][j] * (1 - tx) * (1 - ty) + heights[i][j + 1] * tx * (1 - ty) + heights[i + 1][j] * (1 - tx) * ty + heights[i + 1][j + 1] * tx * ty;
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
    this.mujoco.mj_forward(this.model, data);
    this.map.show(this.mission);
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
    const vT = k.Space ? 0 : fwd * V_MAX, wT = turn * W_MAX;
    const slew = (cur, tgt, rate) => cur + Math.max(-rate * dt, Math.min(rate * dt, tgt - cur));
    this.v = slew(this.v, vT, ACCEL * (vT === 0 ? 2 : 1)); this.w = slew(this.w, wT, ALPHA);
    const wl = (this.v - this.w * TRACK / 2) / WHEEL_R, wr = (this.v + this.w * TRACK / 2) / WHEEL_R;
    data.ctrl[this.acts[0]] = data.ctrl[this.acts[2]] = wl;
    data.ctrl[this.acts[1]] = data.ctrl[this.acts[3]] = wr;

    if (this.step++ % 10 !== 0) return;
    const p = this._pose(data), tick = dt * 10;
    if (this._last) this.dist += Math.hypot(p.x - this._last[0], p.y - this._last[1]);
    this._last = [p.x, p.y];
    // Collisions: chassis touching boulders/terrain (rising edge of the bump sensor).
    const hit = data.sensordata[0] > 40;
    if (hit && !this._bumping) this.bumps++;
    this._bumping = hit;
    // Rollover: if the rover stays upside down/on its side, set it upright where it is.
    this._flipT = p.up < 0.3 ? this._flipT + tick : 0;
    if (this._flipT > 2) { this.rollovers++; this._flipT = 0; this._setPose(data, p.x, p.y, p.yaw); }
    this._missionStep(data, p, tick);
  }

  _dist(p, t) { return Math.hypot(p.x - t[0], p.y - t[1]); }

  _missionStep(data, p, tick) {
    const m = this.mission;
    if (this.phase === 0 && this._dist(p, m.alpha) < REACH) this.phase = 1;
    else if (this.phase === 1) {
      const near = this._dist(p, this.sample) < SAMPLE_REACH && Math.abs(this.v) < 0.4;
      if (near && this.keys.KeyE) this.collect += tick; else if (!this.keys.KeyE) this.collect = Math.max(0, this.collect - tick);
      if (this.collect >= COLLECT_TIME) {
        this.phase = 2; this.hasSample = true;
        data.mocap_pos[this.mc.item * 3 + 2] = -50;                 // sample goes into the rover
      }
    } else if (this.phase === 2 && this._dist(p, this.home) < HOME_REACH) this.phase = 3;
  }

  update(data) {
    const p = this._pose(data);
    const target = this.phase === 0 ? "Drive to the BLUE survey beacon"
      : this.phase === 1 ? (this._dist(p, this.sample) < SAMPLE_REACH ? "At the sample site: stop and hold E to collect" : "Drive to the GREEN sample site")
        : this.phase === 2 ? "Sample secured! Return to the YELLOW beacon at the lander" : "Mission complete";
    const el = document.getElementById("grab-status");
    if (el) {
      const bar = this.phase === 1 && this.collect > 0 ? ` · collecting ${Math.min(100, Math.round(this.collect / COLLECT_TIME * 100))}%` : "";
      el.innerHTML = `<b>${target}</b>${bar}<br>Speed ${Math.abs(this.v).toFixed(1)} m/s · Distance to goal ${this._goalDist(p).toFixed(0)} m`;
    }
    const marks = [
      { x: this.mission.lander[0], y: this.mission.lander[1], color: "#E8E8F0", shape: "square", label: "Lander" },
      { x: this.mission.alpha[0], y: this.mission.alpha[1], color: "#3399FF", label: "Survey", done: this.phase > 0 },
      { x: this.sample[0], y: this.sample[1], color: "#33FFAA", label: "Sample", done: this.hasSample },
      { x: this.home[0], y: this.home[1], color: "#FFD633", label: "Home", done: this.phase >= 3 },
    ];
    this.map.draw(marks, p, `Phase ${Math.min(this.phase + 1, 3)}/3`);
  }

  _goalDist(p) {
    const t = this.phase === 0 ? this.mission.alpha : this.phase === 1 ? this.sample : this.phase === 2 ? this.home : [p.x, p.y];
    return this._dist(p, t);
  }

  /** Chase camera hook used by main.js. */
  chaseTarget(data) {
    const p = this._pose(data);
    return { x: p.x, y: p.y, z: p.z, yaw: p.yaw };
  }

  isSuccess() { return this.phase >= 3; }
  getMetrics() { return { "mission phase": `${Math.min(this.phase, 3)}/3`, "rock hits": this.bumps, rollovers: this.rollovers, "distance (m)": this.dist.toFixed(0) }; }
  score(time, limit, success) {
    let s = 30 * Math.min(this.phase, 3);
    if (success) s += 10 * Math.max(0, 1 - time / limit);
    return Math.max(0, Math.min(100, s - 2 * this.bumps - 5 * this.rollovers));
  }
  dispose() { this.map.hide(); }
}
