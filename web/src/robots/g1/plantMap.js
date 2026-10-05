// Plan view of the plant: equipment, shields, the control terminal, the radiation survey the robot has
// painted along its path (colour = dose rate), the flags it has planted and its own position.
const CSS = `
#plantmap{position:fixed;right:10px;bottom:10px;z-index:1000;background:rgba(8,12,20,.85);border:1px solid rgba(255,214,102,.45);
  border-radius:10px;padding:6px;color:#FFF3C4;font:11px/1.3 -apple-system,'Segoe UI',Arial,sans-serif;text-align:center}
#plantmap[hidden]{display:none}
#plantmap canvas{display:block;border-radius:6px}
#plantmap .cap{margin-top:3px;opacity:.85}
`;
const heat = (gyh) => {                       // dose rate -> colour
  const t = Math.max(0, Math.min(1, (Math.log10(Math.max(gyh, 0.4)) - Math.log10(0.4)) / (Math.log10(4000) - Math.log10(0.4))));
  const stops = [[60, 200, 110], [230, 220, 60], [240, 130, 40], [230, 40, 40], [200, 60, 255]];
  const f = t * (stops.length - 1), i = Math.min(stops.length - 2, Math.floor(f)), u = f - i;
  return `rgb(${stops[i].map((v, k) => Math.round(v + (stops[i + 1][k] - v) * u)).join(",")})`;
};
export class PlantMap {
  constructor() {
    const style = document.createElement("style"); style.textContent = CSS; document.head.appendChild(style);
    this.el = document.createElement("div"); this.el.id = "plantmap"; this.el.hidden = true;
    this.canvas = document.createElement("canvas"); this.el.appendChild(this.canvas);
    this.cap = document.createElement("div"); this.cap.className = "cap"; this.el.appendChild(this.cap);
    document.body.appendChild(this.el);
    this.big = false; this._resize();
    window.addEventListener("keydown", (e) => { if (e.code === "KeyM" && !this.el.hidden) { this.big = !this.big; this._resize(); } });
  }
  _resize() { this.w = this.big ? Math.min(window.innerWidth * 0.6, 700) : 260; this.canvas.width = this.w; this.canvas.height = Math.round(this.w * 0.75); }
  show(cfg) { this.cfg = cfg; this.el.hidden = false; }
  hide() { this.el.hidden = true; }
  _p(x, y) { const [x0, x1, y0, y1] = this.cfg.bounds; return [((x - x0) / (x1 - x0)) * this.canvas.width, this.canvas.height - ((y - y0) / (y1 - y0)) * this.canvas.height]; }
  draw(state, caption) {
    if (!this.cfg || this.el.hidden) return;
    const ctx = this.canvas.getContext("2d"), W = this.canvas.width, H = this.canvas.height, s = W / (this.cfg.bounds[1] - this.cfg.bounds[0]);
    ctx.fillStyle = "#111a26"; ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = "rgba(255,255,255,.07)"; ctx.lineWidth = 1;
    for (let x = this.cfg.bounds[0]; x <= this.cfg.bounds[1]; x += 3) { const [px] = this._p(x, 0); ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, H); ctx.stroke(); }
    for (const sh of this.cfg.shapes) {
      ctx.fillStyle = sh.name === "Concrete shield" ? "#8b8f94" : sh.name === "Control terminal" ? "#2fd18a" : "#3a4658"; ctx.strokeStyle = "rgba(255,255,255,.25)";
      if (sh.t === "circle") { const [cx, cy] = this._p(sh.x, sh.y); ctx.beginPath(); ctx.arc(cx, cy, sh.r * s, 0, 6.283); ctx.fill(); ctx.stroke(); }
      else { const [ax, ay] = this._p(sh.x0, sh.y1), [bx, by] = this._p(sh.x1, sh.y0); ctx.fillRect(ax, ay, bx - ax, by - ay); ctx.strokeRect(ax, ay, bx - ax, by - ay); }
    }
    for (const t of state.trail) { const [px, py] = this._p(t[0], t[1]); ctx.fillStyle = heat(t[2]); ctx.globalAlpha = 0.85; ctx.beginPath(); ctx.arc(px, py, Math.max(2, s * 0.45), 0, 6.283); ctx.fill(); }
    ctx.globalAlpha = 1;
    const [tx, ty] = this._p(this.cfg.terminal[0], this.cfg.terminal[1]);
    ctx.fillStyle = "#2fd18a"; ctx.font = `${this.big ? 12 : 9}px sans-serif`; ctx.fillText("REPORT", tx - 14, ty - 8);
    state.tags.forEach((g, i) => { const [px, py] = this._p(g.x, g.y); ctx.strokeStyle = "#FFD23F"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, py - 12); ctx.stroke(); ctx.fillStyle = "#FFD23F"; ctx.fillRect(px, py - 12, 9, 7); ctx.fillStyle = "#000"; ctx.font = "8px sans-serif"; ctx.fillText(String(i + 1), px + 2, py - 6); });
    if (state.revealed) for (const r of state.revealed) { const [px, py] = this._p(r.x, r.y); ctx.strokeStyle = r.found ? "#7CF2A5" : "#FF6B5B"; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(px, py, this.cfg.tag_radius * s, 0, 6.283); ctx.stroke(); }
    const [rx, ry] = this._p(state.x, state.y), a = -state.yaw, k = this.big ? 9 : 7;
    ctx.save(); ctx.translate(rx, ry); ctx.rotate(a); ctx.fillStyle = "#fff"; ctx.strokeStyle = "#000"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(k, 0); ctx.lineTo(-k * 0.8, k * 0.7); ctx.lineTo(-k * 0.4, 0); ctx.lineTo(-k * 0.8, -k * 0.7); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
    this.cap.textContent = caption + " · M enlarge";
  }
}
