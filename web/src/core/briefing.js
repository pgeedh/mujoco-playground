// Story briefing shown when a mission loads: a short, skippable storyboard with a camera
// fly-through of the scene, cinematic bars and a narrated card per step.
//   steps: [{ kicker, title, text, cam: { pos: [x,y,z], target: [x,y,z] }, stamp?, alarm? }]
// Camera coordinates are three.js world coordinates (y up). Use m2t() to convert from MuJoCo (z up).
export const m2t = (x, y, z) => [x, z, -y];

const CSS = `
#story{position:fixed;inset:0;z-index:1800;pointer-events:none;font:15px/1.55 -apple-system,'Segoe UI',Helvetica,Arial,sans-serif;color:#EAF4FE}
#story[hidden]{display:none}
#story .bar{position:absolute;left:0;right:0;height:11vh;background:#000;transition:height .6s}
#story .bar.t{top:0}#story .bar.b{bottom:0}
#story .vig{position:absolute;inset:0;background:radial-gradient(ellipse at 50% 50%,rgba(0,0,0,0) 55%,rgba(0,0,0,.55) 100%)}
#story .alarm{position:absolute;inset:0;box-shadow:inset 0 0 90px 10px rgba(255,30,30,.55);animation:storyPulse 1.1s ease-in-out infinite;display:none}
#story.alarm .alarm{display:block}
@keyframes storyPulse{0%,100%{opacity:.15}50%{opacity:1}}
#story .card{position:absolute;left:5vw;bottom:15vh;max-width:min(560px,86vw);background:rgba(8,14,24,.8);border:1px solid rgba(167,210,255,.35);
  border-radius:14px;padding:16px 20px;pointer-events:auto;backdrop-filter:blur(3px)}
#story .kick{font:700 11px ui-monospace,Menlo,monospace;letter-spacing:2px;color:#A7D2FF;text-transform:uppercase}
#story h2{margin:4px 0 8px;font:400 34px Impact,'Anton','Arial Narrow Bold',sans-serif;letter-spacing:1px}
#story .txt{font-size:15px;min-height:70px}
#story .stamp{position:absolute;right:5vw;top:15vh;font:700 16px ui-monospace,Menlo,monospace;letter-spacing:3px;color:#FF6B5B;border:2px solid #FF6B5B;border-radius:6px;padding:4px 12px;transform:rotate(-4deg);opacity:.92}
#story .nav{display:flex;align-items:center;gap:10px;margin-top:10px}
#story .dots{display:flex;gap:6px;flex:1}
#story .dots i{width:22px;height:4px;border-radius:2px;background:rgba(255,255,255,.22)}
#story .dots i.on{background:#EAF4FE}
#story button{border:1px solid rgba(167,210,255,.5);background:rgba(255,255,255,.08);color:inherit;border-radius:8px;padding:6px 14px;cursor:pointer;font:inherit}
#story button.go{background:#EAF4FE;color:#0A0A0A;font-weight:700;border-color:#EAF4FE}
#story button:hover{filter:brightness(1.15)}
`;

export class Storyboard {
  constructor(demo) {
    this.demo = demo;
    const style = document.createElement("style"); style.textContent = CSS; document.head.appendChild(style);
    this.el = document.createElement("div"); this.el.id = "story"; this.el.hidden = true;
    this.el.innerHTML = `<div class="vig"></div><div class="alarm"></div><div class="bar t"></div><div class="bar b"></div>
      <div class="stamp" hidden></div>
      <div class="card"><div class="kick"></div><h2></h2><div class="txt"></div>
        <div class="nav"><div class="dots"></div><button data-a="skip">Skip (Esc)</button><button data-a="next" class="go">Next (Space)</button></div></div>`;
    document.body.appendChild(this.el);
    this.q = (s) => this.el.querySelector(s);
    this.el.addEventListener("click", (e) => { const a = e.target.dataset && e.target.dataset.a; if (a === "next") this.next(); if (a === "skip") this.finish(); });
    window.addEventListener("keydown", (e) => {
      if (this.el.hidden) return;
      if (["Space", "Enter", "ArrowRight"].includes(e.code)) { e.preventDefault(); e.stopImmediatePropagation(); this.next(); }
      else if (e.code === "Escape" || e.code === "KeyS") { e.preventDefault(); e.stopImmediatePropagation(); this.finish(); }
      else if (e.code === "ArrowLeft") { e.preventDefault(); this.go(this.i - 1); }
    }, true);
    this.active = false;
  }

  /** Plays the steps; resolves when the player starts the mission (or skips). */
  play(steps) {
    this.steps = steps; this.active = true; this.el.hidden = false;
    const hud = ["controls-help", "minimap", "plantmap", "bench"]; this._hidden = hud.map((id) => document.getElementById(id)).filter(Boolean);
    this._hidden.forEach((e) => { e._was = e.style.visibility; e.style.visibility = "hidden"; });
    this.q(".dots").innerHTML = steps.map(() => "<i></i>").join("");
    this.demo.controls.enabled = false;
    this.cam0 = this._camState();
    this.i = -1;
    return new Promise((res) => { this._done = res; this.next(); this._raf = requestAnimationFrame(this._tick.bind(this)); });
  }

  _camState() { return { pos: this.demo.camera.position.clone(), target: this.demo.controls.target.clone() }; }

  next() { if (this.i >= this.steps.length - 1) return this.finish(); this.go(this.i + 1); }

  go(i) {
    i = Math.max(0, i); if (i === this.i) return;
    this.i = i; const s = this.steps[i];
    this.q(".kick").textContent = s.kicker || ""; this.q("h2").textContent = s.title || "";
    this._type(s.text || "");
    const st = this.q(".stamp"); st.hidden = !s.stamp; st.textContent = s.stamp || "";
    this.el.classList.toggle("alarm", !!s.alarm);
    [...this.q(".dots").children].forEach((d, k) => d.classList.toggle("on", k <= i));
    const last = i === this.steps.length - 1;
    const nb = this.q('[data-a="next"]'); nb.textContent = last ? "Start mission (Space)" : "Next (Space)";
    this.from = this._camState(); this.t0 = performance.now(); this.dur = 2200;
    this.to = { pos: new this.demo.camera.position.constructor(...s.cam.pos), target: new this.demo.camera.position.constructor(...s.cam.target) };
    this.orbit = s.orbit ?? 0.12;
  }

  _type(html) {
    clearTimeout(this._tt); const el = this.q(".txt"); el.innerHTML = html; el.style.visibility = "visible";
    const plain = el.textContent; el.textContent = ""; let n = 0;
    const tick = () => { n += 3; if (n >= plain.length) { el.innerHTML = html; return; } el.textContent = plain.slice(0, n); this._tt = setTimeout(tick, 16); };
    tick();
  }

  _tick(now) {
    if (!this.active) return;
    const t = Math.min(1, (now - this.t0) / this.dur), e = t * t * (3 - 2 * t), d = this.demo;
    d.camera.position.lerpVectors(this.from.pos, this.to.pos, e);
    d.controls.target.lerpVectors(this.from.target, this.to.target, e);
    // slow orbit drift around the target for a cinematic feel
    const a = (now - this.t0) / 1000 * this.orbit, off = d.camera.position.clone().sub(d.controls.target), c = Math.cos(a * 0.15), s = Math.sin(a * 0.15);
    d.camera.position.set(d.controls.target.x + off.x * c - off.z * s, d.camera.position.y, d.controls.target.z + off.x * s + off.z * c);
    d.camera.lookAt(d.controls.target);
    this._raf = requestAnimationFrame(this._tick.bind(this));
  }

  finish() {
    if (!this.active) return;
    this.active = false; cancelAnimationFrame(this._raf); clearTimeout(this._tt);
    this.el.hidden = true; this.el.classList.remove("alarm");
    this._hidden.forEach((e) => { e.style.visibility = e._was || ""; });
    const d = this.demo, env = d.env;
    if (env) { d.controls.enabled = d.cameraMode === "third"; d._setView(env.camera); }
    if (this._done) { const r = this._done; this._done = null; r(); }
  }
}
