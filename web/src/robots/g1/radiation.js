// Radiation field for the nuclear-plant mission (pure maths, no DOM).
// Dose rate at a point = background + sum over leaks of  A / (r^2 + eps), multiplied by an
// attenuation factor for each concrete shield (or the reactor's bio-shield) that the straight
// line from the leak to the point passes through. Units are Gy/h.
const SHIELD_FACTOR = 0.12, REACTOR_FACTOR = 0.05;

function segHitsRect(ax, ay, bx, by, x0, x1, y0, y1) {
  let t0 = 0, t1 = 1; const dx = bx - ax, dy = by - ay;
  for (const [p, q] of [[-dx, ax - x0], [dx, x1 - ax], [-dy, ay - y0], [dy, y1 - ay]]) {
    if (p === 0) { if (q < 0) return false; } else { const r = q / p; if (p < 0) { if (r > t1) return false; if (r > t0) t0 = r; } else { if (r < t0) return false; if (r < t1) t1 = r; } }
  }
  return true;
}
function segHitsCircle(ax, ay, bx, by, cx, cy, r) {
  const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy || 1e-9;
  const t = Math.max(0, Math.min(1, ((cx - ax) * dx + (cy - ay) * dy) / l2));
  return Math.hypot(ax + t * dx - cx, ay + t * dy - cy) < r;
}

export class RadiationField {
  constructor({ shields, reactor, background = 0.4 }) { this.shields = shields; this.reactor = reactor; this.bg = background; this.sources = []; }
  setSources(list) { this.sources = list; }
  /** Dose rate in Gy/h at (x, y, z). */
  doseRate(x, y, z) {
    let d = this.bg;
    for (const s of this.sources) {
      const r2 = (x - s.x) ** 2 + (y - s.y) ** 2 + (z - s.z) ** 2;
      let k = 1;
      for (const [x0, x1, y0, y1] of this.shields) if (segHitsRect(s.x, s.y, x, y, x0, x1, y0, y1)) k *= SHIELD_FACTOR;
      if (segHitsCircle(s.x, s.y, x, y, this.reactor[0], this.reactor[1], this.reactor[2] + 0.4)) k *= REACTOR_FACTOR;
      d += (s.A * k) / (r2 + 0.35);
    }
    return d;
  }
}
