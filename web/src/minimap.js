// Top-down mission map: terrain relief, mission markers, rover pose and trail.
const CSS = `
#minimap{position:fixed;right:10px;bottom:10px;z-index:1000;background:rgba(10,15,25,.8);border:1px solid rgba(255,200,150,.45);
  border-radius:10px;padding:6px;color:#FFE9D6;font:11px/1.3 -apple-system,'Segoe UI',Arial,sans-serif;text-align:center}
#minimap[hidden]{display:none}
#minimap canvas{display:block;border-radius:6px}
#minimap .cap{margin-top:3px;opacity:.8}
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
    this.size = 200;
    this.el.appendChild(this.canvas);
    this.cap = document.createElement("div");
    this.cap.className = "cap";
    this.el.appendChild(this.cap);
    document.body.appendChild(this.el);
    this.big = false;
    this.trail = [];
    this._resize();
    window.addEventListener("keydown", (e) => {
      if (e.code === "KeyM" && !this.el.hidden) { this.big = !this.big; this._resize(); }
    });
  }

  _resize() {
    this.size = this.big ? Math.min(window.innerHeight - 120, 560) : 200;
    this.canvas.width = this.canvas.height = this.size;
    this._base = null;
  }

  /** mission: parsed mission.json (heights, half_extent, zmax). */
  show(mission) { this.m = mission; this.el.hidden = false; this.trail = []; this._base = null; }
  hide() { this.el.hidden = true; }

  _drawBase() {
    const { heights, half_extent: R, zmax } = this.m;
    const n = heights.length, S = this.size;
    const off = document.createElement("canvas"); off.width = off.height = n;
    const ctx = off.getContext("2d"), img = ctx.createImageData(n, n);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
      const h = heights[i][j] / zmax;
      // simple hillshade from the local gradient for relief
      const hx = (heights[i][Math.min(n - 1, j + 1)] - heights[i][Math.max(0, j - 1)]);
      const hy = (heights[Math.min(n - 1, i + 1)][j] - heights[Math.max(0, i - 1)][j]);
      const shade = Math.max(-0.5, Math.min(0.5, (-hx - hy) * 0.35));
      const row = n - 1 - i;                                    // y up on screen
      const p = (row * n + j) * 4;
      img.data[p] = Math.max(0, Math.min(255, 120 + 110 * h + 70 * shade));
      img.data[p + 1] = Math.max(0, Math.min(255, 62 + 70 * h + 45 * shade));
      img.data[p + 2] = Math.max(0, Math.min(255, 38 + 40 * h + 30 * shade));
      img.data[p + 3] = 255;
    }
    ctx.putImageData(img, 0, 0);
    this._base = document.createElement("canvas"); this._base.width = this._base.height = S;
    const b = this._base.getContext("2d"); b.imageSmoothingEnabled = true;
    b.drawImage(off, 0, 0, S, S);
    b.strokeStyle = "rgba(255,255,255,.12)"; b.lineWidth = 1;
    for (let k = 1; k < 4; k++) { b.beginPath(); b.moveTo(k * S / 4, 0); b.lineTo(k * S / 4, S); b.moveTo(0, k * S / 4); b.lineTo(S, k * S / 4); b.stroke(); }
  }

  _px(x, y) { const R = this.m.half_extent, S = this.size; return [(x + R) / (2 * R) * S, S - (y + R) / (2 * R) * S]; }

  /** markers: [{x,y,color,label,shape,done}], rover: {x,y,yaw}. */
  draw(markers, rover, caption) {
    if (!this.m || this.el.hidden) return;
    if (!this._base) this._drawBase();
    const ctx = this.canvas.getContext("2d");
    ctx.drawImage(this._base, 0, 0);
    // trail
    const last = this.trail[this.trail.length - 1];
    if (!last || Math.hypot(last[0] - rover.x, last[1] - rover.y) > 0.6) { this.trail.push([rover.x, rover.y]); if (this.trail.length > 400) this.trail.shift(); }
    ctx.strokeStyle = "rgba(255,255,255,.55)"; ctx.lineWidth = 1.5; ctx.beginPath();
    this.trail.forEach(([x, y], i) => { const [px, py] = this._px(x, y); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
    ctx.stroke();
    if (this.m.rocks) {
      ctx.fillStyle = "rgba(40,22,15,.85)";
      for (const [x, y, s] of this.m.rocks) { const [px, py] = this._px(x, y); ctx.beginPath(); ctx.arc(px, py, Math.max(1.5, s * this.size / (2 * this.m.half_extent) * 1.2), 0, 6.283); ctx.fill(); }
    }
    for (const mk of markers) {
      const [px, py] = this._px(mk.x, mk.y), r = this.big ? 8 : 6;
      ctx.globalAlpha = mk.done ? 0.45 : 1; ctx.fillStyle = mk.color; ctx.strokeStyle = "#0A0A0A"; ctx.lineWidth = 1.5;
      ctx.beginPath();
      if (mk.shape === "square") ctx.rect(px - r, py - r, 2 * r, 2 * r); else ctx.arc(px, py, r, 0, 6.283);
      ctx.fill(); ctx.stroke();
      if (mk.label) { ctx.fillStyle = "#fff"; ctx.font = `${this.big ? 13 : 10}px sans-serif`; ctx.fillText(mk.label, px + r + 3, py + 4); }
      ctx.globalAlpha = 1;
    }
    const [rx, ry] = this._px(rover.x, rover.y), a = -rover.yaw;
    ctx.save(); ctx.translate(rx, ry); ctx.rotate(a);
    ctx.fillStyle = "#FFFFFF"; ctx.strokeStyle = "#0A0A0A"; ctx.lineWidth = 1.5;
    const s = this.big ? 9 : 7;
    ctx.beginPath(); ctx.moveTo(s, 0); ctx.lineTo(-s * 0.8, s * 0.7); ctx.lineTo(-s * 0.4, 0); ctx.lineTo(-s * 0.8, -s * 0.7); ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.restore();
    this.cap.textContent = caption + " · M: enlarge";
  }
}
