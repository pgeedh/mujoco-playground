// Robot-select screen. Only the G1 has a trained policy so far; the other
// robots are listed as coming soon so the roster is visible.
export const ROBOTS = [
  { id: "g1",   name: "Unitree G1",  kind: "Humanoid",  blurb: "Walks with a real pretrained policy. Grab, carry and jump.", ready: true },
  { id: "go2",  name: "Unitree Go2", kind: "Quadruped", blurb: "Four-legged trotting robot dog.",                             ready: false },
  { id: "duck", name: "Micro Duck",  kind: "Biped",     blurb: "Tiny waddling duck robot from the Open Duck project.",       ready: false },
];

const CSS = `
#menu{position:fixed;inset:0;z-index:2000;display:flex;flex-direction:column;align-items:center;justify-content:center;
  background:radial-gradient(ellipse at 50% 30%,rgba(10,30,55,.82),rgba(5,10,20,.95));color:#EAF4FE;
  font:15px/1.5 -apple-system,'Segoe UI',Helvetica,Arial,sans-serif;padding:24px;box-sizing:border-box;text-align:center}
#menu h1{margin:0 0 6px;font:400 44px Impact,'Anton','Arial Narrow Bold',sans-serif;letter-spacing:2px}
#menu p.sub{margin:0 0 28px;color:#C5E3FE;opacity:.85}
#menu .cards{display:flex;gap:18px;flex-wrap:wrap;justify-content:center}
#menu .card{width:220px;padding:20px 18px;border-radius:14px;border:1px solid rgba(167,210,255,.35);background:rgba(255,255,255,.06);
  color:inherit;text-align:left;cursor:pointer;font:inherit;transition:transform .15s,border-color .15s,background .15s}
#menu .card:hover:not(:disabled),#menu .card:focus-visible{transform:translateY(-3px);border-color:#A7D2FF;background:rgba(167,210,255,.14);outline:none}
#menu .card:disabled{opacity:.45;cursor:not-allowed}
#menu .card b{display:block;font-size:19px;margin-bottom:2px}
#menu .card .kind{font-size:12px;letter-spacing:1.5px;text-transform:uppercase;color:#A7D2FF}
#menu .card .blurb{margin:10px 0 14px;font-size:13px;opacity:.85;min-height:58px}
#menu .card .tag{display:inline-block;font-size:11px;font-weight:700;letter-spacing:1px;padding:3px 9px;border-radius:99px;
  background:#EAF4FE;color:#0A0A0A}
#menu .card:disabled .tag{background:transparent;color:#C5E3FE;border:1px solid rgba(197,227,254,.5)}
#menu .hint{margin-top:26px;font-size:13px;opacity:.7}
#change-robot{position:fixed;top:10px;left:10px;z-index:1000;border:0;border-radius:8px;padding:8px 12px;cursor:pointer;
  font:13px -apple-system,'Segoe UI',Arial,sans-serif;color:#fff;background:rgba(0,0,0,.55)}
#change-robot:hover{background:rgba(0,0,0,.75)}
`;

export function initMenu({ onStart, onOpen }) {
  const style = document.createElement("style");
  style.textContent = CSS;
  document.head.appendChild(style);

  const menu = document.createElement("div");
  menu.id = "menu";
  menu.innerHTML = `
    <h1>PICK YOUR ROBOT</h1>
    <p class="sub">Real MuJoCo physics in your browser. Free-roam sandbox: walk around, pick things up, mess about.</p>
    <div class="cards">${ROBOTS.map((r, i) => `
      <button class="card" data-id="${r.id}" ${r.ready ? "" : "disabled"} aria-label="${r.name}${r.ready ? "" : " (coming soon)"}">
        <span class="kind">${r.kind}</span>
        <b>${r.name}</b>
        <div class="blurb">${r.blurb}</div>
        <span class="tag">${r.ready ? `PLAY · press ${i + 1}` : "COMING SOON"}</span>
      </button>`).join("")}
    </div>
    <div class="hint">Esc re-opens this menu</div>`;
  document.body.appendChild(menu);

  const change = document.createElement("button");
  change.id = "change-robot";
  change.textContent = "Change robot (Esc)";
  change.hidden = true;
  document.body.appendChild(change);

  const start = (id) => {
    const robot = ROBOTS.find((r) => r.id === id);
    if (!robot || !robot.ready) return;
    menu.hidden = true;
    change.hidden = false;
    document.activeElement && document.activeElement.blur();
    onStart(id);
  };
  const open = () => {
    menu.hidden = false;
    change.hidden = true;
    onOpen && onOpen();
  };

  menu.querySelectorAll(".card").forEach((c) => c.addEventListener("click", () => start(c.dataset.id)));
  change.addEventListener("click", open);
  window.addEventListener("keydown", (e) => {
    if (menu.hidden) {
      if (e.code === "Escape") open();
      return;
    }
    const idx = ["Digit1", "Digit2", "Digit3"].indexOf(e.code);
    if (idx >= 0) start(ROBOTS[idx].id);
    if (e.code === "Enter") start("g1");
  });
}
