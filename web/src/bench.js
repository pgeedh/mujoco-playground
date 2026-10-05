// Benchmark layer shared by every environment. A controller exposes:
//   hasInput()   - true once the player touches a control (starts the clock)
//   reset(data, seed)
//   getMetrics() - { label: value } shown in the panel and exported
//   isSuccess()  - task complete
//   score()      - 0..100 composite
// The bench owns the clock, the seed, the result card and the JSON export.
const CSS = `
#bench{position:fixed;top:10px;left:50%;transform:translateX(-50%);z-index:1000;min-width:300px;max-width:92vw;
  color:#EAF4FE;background:rgba(10,20,35,.82);border:1px solid rgba(167,210,255,.35);border-radius:12px;padding:10px 14px;
  font:13px/1.45 -apple-system,'Segoe UI',Helvetica,Arial,sans-serif;text-align:center}
#bench[hidden]{display:none}
#bench .t{font-weight:700;letter-spacing:.5px}
#bench .clock{font:600 22px ui-monospace,Menlo,monospace;margin:2px 0}
#bench .m{opacity:.85}
#bench .res{margin:6px 0 2px;font-weight:700;font-size:15px}
#bench .res.ok{color:#7CF2A5}#bench .res.no{color:#FFB3A7}
#bench .btns{margin-top:8px;display:flex;gap:6px;justify-content:center;flex-wrap:wrap}
#bench button{border:1px solid rgba(167,210,255,.45);background:rgba(255,255,255,.08);color:inherit;border-radius:8px;
  padding:4px 10px;cursor:pointer;font:inherit}
#bench button:hover{background:rgba(167,210,255,.22)}
`;

function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
export const makeRng = (seed) => mulberry32(seed);

export class Bench {
  constructor() {
    const style = document.createElement("style");
    style.textContent = CSS;
    document.head.appendChild(style);
    this.el = document.createElement("div");
    this.el.id = "bench";
    this.el.hidden = true;
    document.body.appendChild(this.el);
    this.el.addEventListener("click", (e) => {
      const a = e.target.dataset && e.target.dataset.a;
      if (a === "retry") this.reset(this.seed);
      if (a === "seed") this.reset(this.seed + 1);
      if (a === "export") this.exportJSON();
      e.target.blur && e.target.blur();
    });
    this.env = null; this.ctl = null; this.getData = null;
    const q = new URLSearchParams(location.search).get("seed");
    this.seed = q !== null && !isNaN(+q) ? +q : 0;
  }

  attach(env, controller, getData) {
    this.env = env; this.ctl = controller; this.getData = getData;
    this.el.hidden = false;
    this.reset(this.seed);
  }
  hide() { this.el.hidden = true; }

  reset(seed) {
    this.seed = seed;
    this.el.hidden = false;
    this.state = "ready";        // ready -> running -> done
    this.time = 0;
    this.success = false;
    this.ctl.reset(this.getData(), seed);
    this._render();
  }

  /** Call once per physics step with the step length (s). */
  step(dt) {
    if (!this.ctl || this.state === "done") return;
    if (this.state === "ready") {
      if (!this.ctl.hasInput()) return;
      this.state = "running";
    }
    this.time += dt;
    if (this.ctl.isSuccess()) this._finish(true);
    else if (this.time >= this.env.timeLimit) this._finish(false);
  }

  _finish(success) {
    this.state = "done";
    this.success = success;
    this.finalScore = Math.round(this.ctl.score(this.time, this.env.timeLimit, success));
    this.finalMetrics = this.ctl.getMetrics();
    this._render();
  }

  /** Refresh the live readout (called once per rendered frame). */
  tick() {
    if (!this.ctl || this.el.hidden || this.state === "done") return;
    this._render();
  }

  _render() {
    const m = this.state === "done" ? this.finalMetrics : this.ctl.getMetrics();
    const metrics = Object.entries(m).map(([k, v]) => `${k}: <b>${v}</b>`).join(" &nbsp;·&nbsp; ");
    const clock = this.time.toFixed(1).padStart(5, "0");
    let body = `<div class="t">${this.env.name.toUpperCase()} · ${this.env.task} · seed ${this.seed}</div>`;
    if (this.state === "ready") body += `<div class="clock">READY</div><div class="m">Move to start the clock · limit ${this.env.timeLimit}s</div>`;
    else body += `<div class="clock">${clock}s</div>`;
    if (this.state !== "ready") body += `<div class="m">${metrics}</div>`;
    if (this.state === "done") {
      body += `<div class="res ${this.success ? "ok" : "no"}">${this.success ? "SUCCESS" : "TIME UP"} · score ${this.finalScore}/100</div>`;
    }
    body += `<div class="btns"><button data-a="retry">Retry</button><button data-a="seed">New seed</button><button data-a="export">Export JSON</button></div>`;
    this.el.innerHTML = body;
  }

  result() {
    const done = this.state === "done";
    return {
      benchmark: "mujoco-playground", version: 1,
      env: this.env.id, robot: this.env.name, task: this.env.task,
      seed: this.seed, time_limit_s: this.env.timeLimit,
      finished: done, success: done ? this.success : false,
      time_s: +this.time.toFixed(2),
      score: done ? this.finalScore : Math.round(this.ctl.score(this.time, this.env.timeLimit, false)),
      metrics: done ? this.finalMetrics : this.ctl.getMetrics(),
      timestamp: new Date().toISOString(), user_agent: navigator.userAgent,
    };
  }

  exportJSON() {
    const blob = new Blob([JSON.stringify(this.result(), null, 2)], { type: "application/json" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = `${this.env.id}-seed${this.seed}-${Date.now()}.json`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }
}
