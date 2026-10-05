// Mission map: an orbital image (relief only: craters and ridges, no boulders) with the
// rover's own SLAM occupancy map drawn on top, plus mission markers and trails.
const CSS = `
#minimap{position:fixed;right:10px;bottom:10px;z-index:1000;background:rgba(10,15,25,.82);border:1px solid rgba(255,200,150,.45);
  border-radius:10px;padding:6px;color:#FFE9D6;font:11px/1.3 -apple-system,'Segoe UI',Arial,sans-serif;text-align:center}
#minimap[hidden]{display:none}
#minimap canvas{display:block;border-radius:6px}
#minimap .cap{margin-top:3px;opacity:.85}
#minimap .leg{margin-top:2px;opacity:.7;font-size:10px}
`;

export class Minimap {
  constructor() {
    const style = document.createElement("style");
    style.textContent = CSS;
    document.head.appendChild(style);
    this.el = document.createElement("div");
    this.el.id = "minimap";
    this.el.hidden = true;
    this.canvas = document.createElement("canvas");
    this.el.appendChild(this.canvas);
    this.cap = document.createElement("div"); this.cap.className = "cap"; this.el.appendChild(this.cap);
    this.leg = document.createElement("div"); this.leg.className = "leg";
    this.leg.innerHTML = "<span style='color:#7CF2E6'>■</span> SLAM map &nbsp;<span style='color:#7CF2A5'>—</span> SLAM path &nbsp;<span style='color:#FF8A7A'>—</span> odometry";
    this.el.appendChild(this.leg);
    document.body.appendChild(this.el);
    this.big = false; this.showTruth = false;
    this._resize();
    window.addEventListener("keydown", (e) => {
      if (this.el.hidden) return;
      if (e.code === "KeyM") { this.big = !this.big; this._resize(); }
      if (e.code === "KeyT") this.showTruth = !this.showTruth;
    });
  }

  _resize() {
    this.size = this.big ? Math.min(window.innerHeight - 140, 560) : 200;
    this.canvas.width = this.canvas.height = this.size;
    this._base = null;
  }

  /** mission: parsed mission.json; heights: heights[i][j], i along +y. */
  show(mission, heights) { this.m = mission; this.heights = heights; this.el.hidden = false; this._base = null; this._gridCanvas = null; this._gridAt = 0; }
  hide() { this.el.hidden = true; }

  _drawBase() {
    const heights = this.heights, zmax = this.m.zmax, n = heights.length, S = this.size;
    const off = document.createElement("canvas"); off.width = off.height = n;
    const ctx = off.getContext("2d"), img = ctx.createImageData(n, n);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
      const h = heights[i][j] / zmax;
      const hx = heights[i][Math.min(n - 1, j + 2)] - heights[i][Math.max(0, j - 2)];
      const hy = heights[Math.min(n - 1, i + 2)][j] - heights[Math.max(0, i - 2)][j];
      const shade = Math.max(-0.5, Math.min(0.5, (-hx - hy) * 0.5));
      const p = ((n - 1 - i) * n + j) * 4;                      // y up on screen
      img.data[p] = Math.max(0, Math.min(255, 105 + 95 * h + 60 * shade));
      img.data[p + 1] = Math.max(0, Math.min(255, 58 + 60 * h + 40 * shade));
      img.data[p + 2] = Math.max(0, Math.min(255, 36 + 35 * h + 26 * shade));
      img.data[p + 3] = 255;
    }
    ctx.putImageData(img, 0, 0);
    this._base = document.createElement("canvas"); this._base.width = this._base.height = S;
    const b = this._base.getContext("2d"); b.imageSmoothingEnabled = true; b.drawImage(off, 0, 0, S, S);
    b.strokeStyle = "rgba(255,255,255,.12)"; b.lineWidth = 1;
    for (let k = 1; k < 4; k++) { b.beginPath(); b.moveTo(k * S / 4, 0); b.lineTo(k * S / 4, S); b.moveTo(0, k * S / 4); b.lineTo(S, k * S / 4); b.stroke(); }
  }

  _drawGrid(grid) {
    const n = grid.n;
    if (!this._gridCanvas || this._gridCanvas.width !== n) { this._gridCanvas = document.createElement("canvas"); this._gridCanvas.width = this._gridCanvas.height = n; }
    const ctx = this._gridCanvas.getContext("2d"), img = ctx.createImageData(n, n), l = grid.l;
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
      const v = l[i * n + j], p = ((n - 1 - i) * n + j) * 4;
      if (v > 1) { img.data[p] = 124; img.data[p + 1] = 242; img.data[p + 2] = 230; img.data[p + 3] = 235; }
      else if (v < -0.6) { img.data[p] = 255; img.data[p + 1] = 255; img.data[p + 2] = 255; img.data[p + 3] = Math.min(70, 20 + -v * 18); }
    }
    ctx.putImageData(img, 0, 0);
  }

  _px(x, y) { const R = this.m.half_extent, S = this.size; return [(x + R) / (2 * R) * S, S - (y + R) / (2 * R) * S]; }

  _trail(ctx, pts, color, width) {
    if (pts.length < 2) return;
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
    pts.forEach(([x, y], i) => { const [px, py] = this._px(x, y); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
    ctx.stroke();
  }

  _arrow(ctx, x, y, yaw, fill, stroke, scale = 1) {
    const [rx, ry] = this._px(x, y), s = (this.big ? 9 : 7) * scale;
    ctx.save(); ctx.translate(rx, ry); ctx.rotate(-yaw);
    ctx.fillStyle = fill; ctx.strokeStyle = stroke; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(s, 0); ctx.lineTo(-s * 0.8, s * 0.7); ctx.lineTo(-s * 0.4, 0); ctx.lineTo(-s * 0.8, -s * 0.7); ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.restore();
  }

  /** state: { grid, est, truth, trailEst, trailOdo, trailTruth } */
  draw(markers, state, caption) {
    if (!this.m || this.el.hidden) return;
    if (!this._base) this._drawBase();
    const ctx = this.canvas.getContext("2d"), S = this.size;
    ctx.drawImage(this._base, 0, 0);
    const now = performance.now();
    if (state.grid && (!this._gridCanvas || now - this._gridAt > 400)) { this._drawGrid(state.grid); this._gridAt = now; }
    if (this._gridCanvas) { ctx.imageSmoothingEnabled = false; ctx.drawImage(this._gridCanvas, 0, 0, S, S); }
    this._trail(ctx, state.trailOdo, "rgba(255,138,122,.8)", 1.3);
    this._trail(ctx, state.trailEst, "rgba(124,242,165,.95)", 1.8);
    if (this.showTruth) this._trail(ctx, state.trailTruth, "rgba(255,255,255,.6)", 1.2);
    for (const mk of markers) {
      const [px, py] = this._px(mk.x, mk.y), r = this.big ? 8 : 6;
      ctx.globalAlpha = mk.done ? 0.45 : 1; ctx.fillStyle = mk.color; ctx.strokeStyle = "#0A0A0A"; ctx.lineWidth = 1.5;
      ctx.beginPath();
      if (mk.shape === "square") ctx.rect(px - r, py - r, 2 * r, 2 * r); else ctx.arc(px, py, r, 0, 6.283);
      ctx.fill(); ctx.stroke();
      if (mk.label) { ctx.fillStyle = "#fff"; ctx.font = `${this.big ? 13 : 10}px sans-serif`; ctx.fillText(mk.label, px + r + 3, py + 4); }
      ctx.globalAlpha = 1;
    }
    if (this.showTruth && state.truth) this._arrow(ctx, state.truth.x, state.truth.y, state.truth.yaw, "rgba(255,255,255,.25)", "#fff", 1.1);
    this._arrow(ctx, state.est.x, state.est.y, state.est.yaw, "#7CF2A5", "#0A0A0A");
    this.cap.textContent = caption + " · M enlarge · T true path";
  }
}
