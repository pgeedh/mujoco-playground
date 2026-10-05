// "Reactor 3 leak": the G1 walks the plant with a dosimeter, finds the radioactive leaks among
// several steaming pipe couplings, tags them with flags (F) and reports at the control terminal
// (Enter) before its electronics take too much dose.
import * as THREE from "three";
import { makeRng } from "../../core/bench.js";
import { RadiationField } from "./radiation.js";
import { PlantMap } from "./plantMap.js";

const N_REAL = 3, MAX_TAGS = 3;
const SENSOR_Z = 0.8;                      // dosimeter height on the torso (m)
const STEAM_N = 360;

export class PlantMission {
  constructor() {
    this.map = new PlantMap();
    this._prev = {}; this.keys = null;
  }

  async load() { this.cfg = await (await fetch("./assets/scenes/g1_plant/plant_mission.json")).json(); }

  bind(model, mujoco, ctx, ctl) {
    this.model = model; this.mujoco = mujoco; this.ctx = ctx; this.ctl = ctl; this.keys = ctl.keys;
    const scene = ctx && ctx.scene; if (!scene) return;
    if (!this.visuals) {
      this.visuals = new THREE.Group();
      this.flags = new THREE.Group(); this.visuals.add(this.flags);
      const g = new THREE.BufferGeometry(); g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(STEAM_N * 3), 3));
      this.steam = new THREE.Points(g, new THREE.PointsMaterial({ color: 0xe8eef2, size: 0.35, sizeAttenuation: true, transparent: true, opacity: 0.34, depthWrite: false }));
      this.steam.frustumCulled = false; this.visuals.add(this.steam);
      this.steamVel = new Float32Array(STEAM_N * 3); this.steamAge = new Float32Array(STEAM_N).fill(99); this.steamNext = 0;
      this.reactorLight = new THREE.PointLight(0xffa040, 0, 18, 1.2); this.reactorLight.position.set(12, 6.5, 0); this.visuals.add(this.reactorLight);
      this.alarmLight = new THREE.PointLight(0xff2a1a, 0, 9, 1.4); this.alarmLight.position.set(0.2, 1.9, 4.5); this.visuals.add(this.alarmLight);
    }
    scene.add(this.visuals);
  }
  dispose() { this.map.hide(); if (this.visuals && this.visuals.parent) this.visuals.parent.remove(this.visuals); }

  reset(data, seed) {
    this.field = new RadiationField({ shields: this.cfg.shields, reactor: this.cfg.reactor });
    const rng = makeRng(seed * 2654435 + 11);
    const order = this.cfg.candidates.map((c, i) => [rng(), i]).sort((a, b) => a[0] - b[0]).map((x) => x[1]);
    const real = new Set(order.slice(0, N_REAL));
    this.sources = this.cfg.candidates.filter((_, i) => real.has(i)).map((c) => ({ ...c, A: 2400 + rng() * 1200, found: false }));
    this.field.setSources(this.sources);
    this.tags = []; this.trail = []; this._trailAt = null;
    this.absorbed = 0; this.doseRate = this.field.bg; this.peak = this.doseRate; this.dead = false;
    this.finished = false; this.success = false; this.found = 0; this.reveal = null; this.msg = "";
    this.clock = 0; this._frame = 0;
    if (this.flags) while (this.flags.children.length) this.flags.remove(this.flags.children[0]);
    if (this.steamAge) this.steamAge.fill(99);
    this.map.show(this.cfg);
  }

  _pose(data) {
    const x = data.qpos[0], y = data.qpos[1], qw = data.qpos[3], qz = data.qpos[6], qx = data.qpos[4], qy = data.qpos[5];
    return { x, y, yaw: Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)) };
  }
  _edge(code) { const d = !!this.keys[code], e = d && !this._prev[code]; this._prev[code] = d; return e; }

  /** Per physics step. */
  step(data, dt) {
    if (this.finished || this.dead) return;
    const p = this._pose(data), r = this.field.doseRate(p.x, p.y, SENSOR_Z);
    this.doseRate = r; this.peak = Math.max(this.peak, r);
    this.absorbed += (r * dt) / 3600;
    if (this.absorbed >= this.cfg.budget_gy) { this.dead = true; this.msg = "Dose limit exceeded: the robot's electronics have failed."; }
    this.clock += dt;
  }

  /** Per rendered frame: keys, map, audio, HUD, effects. */
  update(data, dtSec = 0.016) {
    if (!this.cfg || !this.sources) return;
    const p = this._pose(data);
    if (!this.finished && !this.dead) {
      if (this._edge("KeyF") && this.tags.length < MAX_TAGS) { this.tags.push({ x: p.x, y: p.y, dose: this.doseRate }); this._addFlag(p.x, p.y, this.tags.length); }
      if (this._edge("KeyG") && this.tags.length) { this.tags.pop(); this.flags.remove(this.flags.children[this.flags.children.length - 1]); }
      if (this._edge("Enter") || this._edge("KeyR")) this._report(p);
    }
    // survey trail: one dot every 0.45 m painted with the dose rate measured there
    if (!this._trailAt || Math.hypot(p.x - this._trailAt[0], p.y - this._trailAt[1]) > 0.45) { this.trail.push([p.x, p.y, this.doseRate]); this._trailAt = [p.x, p.y]; if (this.trail.length > 700) this.trail.shift(); }
    this.map.draw({ x: p.x, y: p.y, yaw: p.yaw, trail: this.trail, tags: this.tags, revealed: this.reveal }, this.finished ? "Report filed" : `Leaks found: ${this.found}/${N_REAL}`);
    this._hud(p);
    this._effects(dtSec);
  }

  _report(p) {
    const term = this.cfg.terminal;
    if (Math.hypot(p.x - term[0], p.y - term[1]) > this.cfg.report_radius) { this.msg = "Walk to the control terminal (green pad) to file the report."; return; }
    if (!this.tags.length) { this.msg = "Nothing to report: plant at least one flag (F) on a suspected leak."; return; }
    const free = this.tags.map(() => true); let found = 0;
    for (const s of this.sources) {
      let best = -1, bd = this.cfg.tag_radius;
      this.tags.forEach((t, i) => { const d = Math.hypot(t.x - s.x, t.y - s.y); if (free[i] && d < bd) { bd = d; best = i; } });
      s.found = best >= 0; if (best >= 0) { free[best] = false; found++; }
    }
    this.found = found; this.finished = true; this.success = found === N_REAL;
    this.reveal = this.sources.map((s) => ({ x: s.x, y: s.y, found: s.found }));
    this.msg = this.success ? "Report accepted: all leaks located. Plant crews are on their way." : `Report filed: ${found} of ${N_REAL} leaks located.`;
  }

  _addFlag(x, y, n) {
    const g = new THREE.Group(), col = [0xffd23f, 0xffa23f, 0xff6b3f][n - 1] || 0xffd23f;
    g.add(new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1.4, 8), new THREE.MeshBasicMaterial({ color: 0xdddddd })).translateY(0.7));
    g.add(new THREE.Mesh(new THREE.PlaneGeometry(0.5, 0.32), new THREE.MeshBasicMaterial({ color: col, side: THREE.DoubleSide })).translateY(1.25).translateX(0.25));
    g.position.set(x, 0, -y); this.flags.add(g);
  }

  _hud(p) {
    const el = document.getElementById("grab-status"); if (!el) return;
    const d = this.doseRate, bars = Math.max(0, Math.min(10, Math.round((Math.log10(Math.max(d, 0.4) / 0.4) / Math.log10(4000 / 0.4)) * 10)));
    const gauge = "█".repeat(bars) + "░".repeat(10 - bars), term = Math.hypot(p.x - this.cfg.terminal[0], p.y - this.cfg.terminal[1]);
    const lead = this.msg ? `<b>${this.msg}</b><br>` : "<b>Find the radioactive leaks, flag them (F), then report at the terminal (Enter).</b><br>";
    el.innerHTML = lead + `Dose rate <b>${d < 10 ? d.toFixed(1) : Math.round(d)} Gy/h</b> ${gauge} · Absorbed <b>${this.absorbed.toFixed(2)} / ${this.cfg.budget_gy} Gy</b><br>Flags ${this.tags.length}/${MAX_TAGS} · Terminal ${term.toFixed(0)} m`;
  }

  _effects(dt) {
    if (!this.visuals) return;
    const t = performance.now() / 1000;
    this.reactorLight.intensity = 1.2 + 0.5 * Math.sin(t * 2.1) + 0.3 * Math.sin(t * 7.3);
    this.alarmLight.intensity = 1 + 4 * Math.max(0, Math.sin(t * 4));
    // steam venting at every candidate coupling (radioactive or not, so steam alone gives nothing away)
    const sp = this.steam.geometry.attributes.position, dts = Math.min(0.05, dt);
    for (let k = 0; k < 6; k++) { if (Math.random() < 0.55) { const c = this.cfg.candidates[Math.floor(Math.random() * this.cfg.candidates.length)], i = this.steamNext++ % STEAM_N;
      sp.setXYZ(i, c.x + (Math.random() - 0.5) * 0.1, c.z, -(c.y + (Math.random() - 0.5) * 0.1)); this.steamVel[i * 3] = (Math.random() - 0.5) * 0.35; this.steamVel[i * 3 + 1] = 0.5 + Math.random() * 0.5; this.steamVel[i * 3 + 2] = (Math.random() - 0.5) * 0.35; this.steamAge[i] = 0; } }
    for (let i = 0; i < STEAM_N; i++) {
      this.steamAge[i] += dts;
      if (this.steamAge[i] > 2.6) { sp.setY(i, -50); continue; }
      sp.setXYZ(i, sp.getX(i) + this.steamVel[i * 3] * dts, sp.getY(i) + this.steamVel[i * 3 + 1] * dts, sp.getZ(i) + this.steamVel[i * 3 + 2] * dts);
    }
    sp.needsUpdate = true;
  }

  // ---- benchmark interface ----
  isFinished() { return this.finished; }
  isSuccess() { return this.finished && this.success; }
  isFailed() { return this.dead; }
  failReason() { return "electronics fried by radiation"; }
  getMetrics() {
    return { "leaks found": `${this.found}/${N_REAL}`, flags: `${this.tags.length}/${MAX_TAGS}`, "dose (Gy)": `${this.absorbed.toFixed(1)}/${this.cfg.budget_gy}`, "peak rate (Gy/h)": Math.round(this.peak), falls: this.ctl.falls };
  }
  score(time, limit, success) {
    let s = 22 * this.found + (this.finished ? 9 : 0);
    if (success) s += 10 * Math.max(0, 1 - time / limit) + 15 * Math.max(0, 1 - this.absorbed / this.cfg.budget_gy);
    return Math.max(0, Math.min(100, s - 3 * this.ctl.falls));
  }
}
