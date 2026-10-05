// Geiger-counter audio: Poisson clicks whose rate follows the dose rate (WebAudio, no assets).
export class Geiger {
  constructor() { this.ctx = null; this.muted = false; this.noise = null; }
  ensure() {
    try {
      if (!this.ctx) this.ctx = new (window.AudioContext || window.webkitAudioContext)();
      if (this.ctx.state === "suspended") this.ctx.resume();
      if (!this.noise) {
        const n = Math.floor(this.ctx.sampleRate * 0.012), buf = this.ctx.createBuffer(1, n, this.ctx.sampleRate), d = buf.getChannelData(0);
        for (let i = 0; i < n; i++) d[i] = (Math.random() * 2 - 1) * Math.pow(1 - i / n, 3);
        this.noise = buf;
      }
    } catch { this.ctx = null; }
  }
  /** rate: dose rate in Gy/h; dt: seconds since the last call. */
  update(rate, dt) {
    if (!this.ctx || this.muted) return;
    const cps = Math.max(0.8, Math.min(80, 0.8 + rate * 0.06));
    let n = 0, L = Math.exp(-cps * dt), p = 1;
    do { n++; p *= Math.random(); } while (p > L && n < 12);
    for (let i = 0; i < n - 1; i++) this._click();
  }
  _click() {
    const src = this.ctx.createBufferSource(), g = this.ctx.createGain();
    src.buffer = this.noise; g.gain.value = 0.18 + Math.random() * 0.1;
    src.connect(g); g.connect(this.ctx.destination);
    src.start(this.ctx.currentTime + Math.random() * 0.03);
  }
  stop() { if (this.ctx && this.ctx.state === "running") this.ctx.suspend(); }
}
