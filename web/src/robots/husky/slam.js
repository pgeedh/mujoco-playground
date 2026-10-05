// Lidar + wheel-odometry + scan-matching SLAM for the Husky (2D, in the world frame).
// Pure JavaScript with no DOM access so it can be unit-tested outside the browser.
//
//   Lidar        a 270-degree planar scanner that ray-marches the real terrain heightfield and
//                boulders (range noise + dropouts), like a SICK LMS1xx on the Husky's top plate.
//   Odometry     dead reckoning from wheel speeds with a skid-steer yaw model that is deliberately
//                imperfect, so it drifts the way real uncalibrated odometry does.
//   Scan match   correlative scan matching of each scan against the occupancy map built so far
//                (the Hector-SLAM idea). No loop closure.
//   Occupancy    log-odds grid, updated from each scan at the SLAM pose estimate.
import { makeRng } from "../../core/bench.js";

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

export class OccGrid {
  constructor(half, res) {
    this.half = half; this.res = res; this.n = Math.ceil((2 * half) / res);
    this.l = new Float32Array(this.n * this.n);       // log-odds, 0 = unknown
    this.occCount = 0;
  }
  cell(x, y) {
    const i = Math.floor((y + this.half) / this.res), j = Math.floor((x + this.half) / this.res);
    return i < 0 || j < 0 || i >= this.n || j >= this.n ? -1 : i * this.n + j;
  }
  at(x, y) { const c = this.cell(x, y); return c < 0 ? 0 : this.l[c]; }
  /** Bilinear sample of occupied-ness (0..1) for sub-cell scan matching. */
  occ(x, y) {
    const fx = (x + this.half) / this.res - 0.5, fy = (y + this.half) / this.res - 0.5, j = Math.floor(fx), i = Math.floor(fy);
    if (i < 0 || j < 0 || i >= this.n - 1 || j >= this.n - 1) return 0;
    const tx = fx - j, ty = fy - i, n = this.n, l = this.l, o = (v) => (v > 0 ? Math.min(1, v / 2) : 0);
    return o(l[i * n + j]) * (1 - tx) * (1 - ty) + o(l[i * n + j + 1]) * tx * (1 - ty) + o(l[(i + 1) * n + j]) * (1 - tx) * ty + o(l[(i + 1) * n + j + 1]) * tx * ty;
  }
  _bump(c, d) {
    const before = this.l[c], after = Math.max(-4, Math.min(4, before + d));
    this.l[c] = after;
    if (before <= 0.5 && after > 0.5) this.occCount++; else if (before > 0.5 && after <= 0.5) this.occCount--;
  }
  /** Integrate one scan taken at `pose` (the SLAM estimate). */
  integrate(pose, ranges, angles, maxRange) {
    const step = this.res * 0.6, ox = pose.x + 0.46 * Math.cos(pose.yaw), oy = pose.y + 0.46 * Math.sin(pose.yaw);
    for (let b = 0; b < ranges.length; b++) {
      const r = ranges[b], hit = r < maxRange - 0.05, a = pose.yaw + angles[b], ca = Math.cos(a), sa = Math.sin(a);
      const lim = hit ? r - this.res : r;
      let last = -1;
      for (let d = 0.3; d < lim; d += step) {
        const c = this.cell(ox + ca * d, oy + sa * d);
        if (c >= 0 && c !== last) { this._bump(c, -0.35); last = c; }
      }
      if (hit) { const c = this.cell(ox + ca * r, oy + sa * r); if (c >= 0) this._bump(c, 0.9); }
    }
  }
}

export class Lidar {
  constructor({ beams = 270, fov = (270 * Math.PI) / 180, range = 20, mount = 0.46, height = 0.05 } = {}) {
    this.beams = beams; this.range = range; this.mount = mount; this.height = height;
    this.angles = new Float32Array(beams);
    for (let i = 0; i < beams; i++) this.angles[i] = -fov / 2 + (fov * i) / (beams - 1);
  }
  /** world: { ground(x,y), rocks: [[x,y,s,sy,sz,yaw]...] }. truth: { x, y, z, yaw } with z = base height. */
  scan(truth, world, rng) {
    const ranges = new Float32Array(this.beams), zl = truth.z + this.height;
    const ox = truth.x + this.mount * Math.cos(truth.yaw), oy = truth.y + this.mount * Math.sin(truth.yaw);
    for (let b = 0; b < this.beams; b++) {
      const a = truth.yaw + this.angles[b], ca = Math.cos(a), sa = Math.sin(a);
      let prev = 0.5, hit = this.range;
      for (let d = 0.5; d <= this.range; d += 0.15) {
        if (this._blocked(ox + ca * d, oy + sa * d, zl, world)) {
          let lo = prev, hi = d;                       // refine the hit by bisection
          for (let k = 0; k < 6; k++) { const mid = (lo + hi) / 2; if (this._blocked(ox + ca * mid, oy + sa * mid, zl, world)) hi = mid; else lo = mid; }
          hit = hi; break;
        }
        prev = d;
      }
      if (hit < this.range) hit += rng.gauss() * 0.02;     // range noise
      if (hit < this.range && rng.next() < 0.015) hit = this.range;   // dropout
      ranges[b] = Math.max(0.3, Math.min(this.range, hit));
    }
    return ranges;
  }
  _blocked(x, y, zl, world) {
    // Only landmarks that are fixed in the world: steep walls, the lander and boulders. (A gently
    // rolling slope would cut the flat scan plane at a different place from every position.)
    const g = world.ground, e = 0.25;
    if (Math.max(Math.abs(g(x + e, y) - g(x - e, y)), Math.abs(g(x, y + e) - g(x, y - e))) / (2 * e) > 0.7 && g(x, y) > zl - 0.5) return true;
    for (const b of world.boxes || []) {
      if (Math.abs(x - b.x) < b.hx && Math.abs(y - b.y) < b.hy && zl > b.z0 && zl < b.z1) return true;
    }
    for (const [rx, ry, s, sy, sz, yaw] of world.rocksNear(x, y)) {
      const dx = x - rx, dy = y - ry, c = Math.cos(yaw), sn = Math.sin(yaw);
      const u = (dx * c + dy * sn) / s, v = (-dx * sn + dy * c) / (s * sy);
      const zc = world.ground(rx, ry) + s * sz * 0.35, w = (zl - zc) / (s * sz);
      if (u * u + v * v + w * w < 1) return true;
    }
    return false;
  }
}

/** Indexes boulders in a coarse grid so lidar rays only test nearby ones. */
export function makeWorld(ground, rocks, half, boxes = []) {
  const cell = 4, n = Math.ceil((2 * half) / cell), buckets = Array.from({ length: n * n }, () => []);
  for (const r of rocks) {
    const reach = r[2] * Math.max(1, r[3]) + 0.1;
    for (let gx = Math.floor((r[0] - reach + half) / cell); gx <= Math.floor((r[0] + reach + half) / cell); gx++)
      for (let gy = Math.floor((r[1] - reach + half) / cell); gy <= Math.floor((r[1] + reach + half) / cell); gy++)
        if (gx >= 0 && gy >= 0 && gx < n && gy < n) buckets[gy * n + gx].push(r);
  }
  const none = [];
  return { ground, rocks, boxes, rocksNear: (x, y) => { const gx = Math.floor((x + half) / cell), gy = Math.floor((y + half) / cell); return gx < 0 || gy < 0 || gx >= n || gy >= n ? none : buckets[gy * n + gx]; } };
}

export class SlamSystem {
  constructor({ world, half, res = 0.15, seed = 1 }) {
    this.world = world; this.half = half; this.res = res;
    this.lidar = new Lidar();
    this.seed = seed;
  }
  reset(truth) {
    const r = makeRng(this.seed * 7907 + 3);
    this.rng = { next: r, gauss: () => { let u = 0; for (let i = 0; i < 6; i++) u += r(); return (u - 3) * 1.41; } };
    this.grid = new OccGrid(this.half, this.res);
    this.est = { x: truth.x, y: truth.y, yaw: truth.yaw };
    this.odo = { x: truth.x, y: truth.y, yaw: truth.yaw };
    this.scanClock = 0; this.scans = 0; this.lastHits = []; this.lastRanges = null;
    this.errSlam = 0; this.errOdo = 0; this.sumSlam = 0; this.sumOdo = 0; this.samples = 0; this.corrected = 0;
    this.gyroBias = (this.rng.next() - 0.5) * 0.002;      // gyro bias, up to ~0.06 deg/s (tactical-grade MEMS)
  }
  /** Wheel-odometry increment. wl/wr are wheel angular speeds (rad/s). */
  _odometry(dt, wl, wr) {
    const R = 0.1651, TRACK_EFF = 0.555 * 1.3;                  // uncalibrated skid-steer track: yaw rate is over-estimated
    const v = (R * 1.012 * (wl + wr)) / 2 * (1 + this.rng.gauss() * 0.012);
    const w = (R * (wr - wl)) / TRACK_EFF + this.gyroBias + this.rng.gauss() * 0.01;
    return { dist: v * dt, dyaw: w * dt };
  }
  _advance(p, inc) { const mid = p.yaw + inc.dyaw / 2; p.x += inc.dist * Math.cos(mid); p.y += inc.dist * Math.sin(mid); p.yaw = wrap(p.yaw + inc.dyaw); }

  /** Inertial/visual odometry like a Mars rover's: speed over the ground from a downward camera
   *  (visual odometry, immune to wheel slip) and yaw rate from a gyro with a small bias. */
  _inertial(dt, imu) {
    const v = imu.v * (1 + this.rng.gauss() * 0.02) + this.rng.gauss() * 0.015;
    const w = imu.w + this.gyroBias + this.rng.gauss() * 0.004;
    return { dist: v * dt, dyaw: w * dt };
  }

  /** Call every control tick with the true pose (for the lidar), wheel speeds (naive wheel odometry
   *  baseline) and imu = { v, w } measured forward speed and yaw rate (what the SLAM prediction uses). */
  update(dt, truth, wl, wr, imu) {
    const wheel = this._odometry(dt, wl, wr);
    this._advance(this.odo, wheel);
    this._advance(this.est, imu ? this._inertial(dt, imu) : wheel);
    this.scanClock += dt;
    if (this.scanClock >= 0.125) {
      this.scanClock = 0;
      const ranges = this.lidar.scan(truth, this.world, this.rng);
      this._correct(ranges);
      this.grid.integrate(this.est, ranges, this.lidar.angles, this.lidar.range);
      this.lastRanges = ranges; this.scans++;
      this.lastHits = [];
      for (let b = 0; b < ranges.length; b += 1) if (ranges[b] < this.lidar.range - 0.05) {
        const a = truth.yaw + this.lidar.angles[b], ox = truth.x + 0.46 * Math.cos(truth.yaw), oy = truth.y + 0.46 * Math.sin(truth.yaw);
        this.lastHits.push([ox + Math.cos(a) * ranges[b], oy + Math.sin(a) * ranges[b]]);
      }
    }
    this.errSlam = Math.hypot(this.est.x - truth.x, this.est.y - truth.y);
    this.errOdo = Math.hypot(this.odo.x - truth.x, this.odo.y - truth.y);
    this.sumSlam += this.errSlam * this.errSlam; this.sumOdo += this.errOdo * this.errOdo; this.samples++;
  }

  /** Correlative scan matching of this scan against the map so far, around the odometry prediction. */
  _correct(ranges) {
    const g = this.grid, L = this.lidar;
    if (g.occCount < 10) return;                                    // not enough map yet
    const pts = [];
    for (let b = 0; b < ranges.length; b += 3) if (ranges[b] < L.range - 0.05) pts.push([L.angles[b], ranges[b]]);
    if (pts.length < 6) return;
    const score = (x, y, yaw) => {
      const ox = x + L.mount * Math.cos(yaw), oy = y + L.mount * Math.sin(yaw);
      let s = 0;
      for (const [a, r] of pts) { const t = yaw + a; s += g.occ(ox + Math.cos(t) * r, oy + Math.sin(t) * r); }
      return s / pts.length;
    };
    this._lastScore = score;                                       // exposed for diagnostics
    const base = { ...this.est }, s0 = score(base.x, base.y, base.yaw);
    let best = { x: base.x, y: base.y, yaw: base.yaw, s: s0 };
    const search = (cx, cy, cyaw, rx, nx, ryaw, nyaw) => {
      for (let i = -nx; i <= nx; i++) for (let j = -nx; j <= nx; j++) for (let k = -nyaw; k <= nyaw; k++) {
        const x = cx + (i * rx) / nx, y = cy + (j * rx) / nx, yaw = cyaw + (k * ryaw) / nyaw;
        const prior = ((x - base.x) ** 2 + (y - base.y) ** 2) / 0.04 + wrap(yaw - base.yaw) ** 2 / 0.003;
        const s = score(x, y, yaw) - 0.03 * prior;
        if (s > best.s) best = { x, y, yaw, s };
      }
    };
    search(base.x, base.y, base.yaw, 0.12, 3, 0.05, 3);              // coarse: +/-12 cm, +/-3 degrees
    search(best.x, best.y, best.yaw, 0.03, 2, 0.012, 2);             // fine
    if (best.s - s0 > 0.02) { this.est = { x: best.x, y: best.y, yaw: wrap(best.yaw) }; this.corrected++; }
  }

  get rmsSlam() { return this.samples ? Math.sqrt(this.sumSlam / this.samples) : 0; }
  get rmsOdo() { return this.samples ? Math.sqrt(this.sumOdo / this.samples) : 0; }
}
