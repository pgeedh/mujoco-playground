"""Builds the nuclear power plant for the Unitree G1 investigation mission:
web/assets/scenes/g1_plant/plant.xml (the MuJoCo world) and plant_mission.json (leak candidates,
radiation shields, map shapes) so the browser game and the world share one source of truth.

Everything the robot could trip over is visual-only: the pretrained walking policy was trained on flat
ground, so only the outer walls and the floor collide."""
import json, math
from pathlib import Path

OUT = Path("web/assets/scenes/g1_plant"); OUT.mkdir(parents=True, exist_ok=True)
X0, X1, Y0, Y1 = -3.0, 21.0, -9.0, 9.0
g = []                          # worldbody children
shapes = []                     # footprints for the in-game plan map
def vis(t, size, pos, mat, extra=""): g.append(f'    <geom type="{t}" size="{size}" pos="{pos}" material="{mat}" contype="0" conaffinity="0" {extra}/>')
def pipe(a, b, r, mat, kind="cylinder"): g.append(f'    <geom type="{kind}" size="{r}" fromto="{a[0]} {a[1]} {a[2]} {b[0]} {b[1]} {b[2]}" material="{mat}" contype="0" conaffinity="0"/>')
def tank(x, y, r, h, mat, name, cap=True):
    vis("cylinder", f"{r} {h/2}", f"{x} {y} {h/2}", mat)
    if cap: vis("sphere", f"{r}", f"{x} {y} {h}", mat)
    shapes.append({"t": "circle", "x": x, "y": y, "r": r, "name": name})

# --- shell: floor and perimeter walls (these collide) ---
walls = [(f"{(X0+X1)/2} {Y1+0.2} 2.5", f"{(X1-X0)/2+0.4} 0.2 2.5"), (f"{(X0+X1)/2} {Y0-0.2} 2.5", f"{(X1-X0)/2+0.4} 0.2 2.5"),
         (f"{X1+0.2} 0 2.5", f"0.2 {(Y1-Y0)/2} 2.5"), (f"{X0-0.2} 0 2.5", f"0.2 {(Y1-Y0)/2} 2.5")]
for p, s in walls: g.append(f'    <geom type="box" size="{s}" pos="{p}" material="wall"/>')
# --- hazard lane markings guiding the way in (visual) ---
for i in range(0, 22):
    x = -1.0 + i * 1.0
    vis("box", "0.25 0.06 0.002", f"{x} -0.0 0.002", "hazard_y" if i % 2 == 0 else "hazard_k")
for y in (-1.2, 1.2): vis("box", "11 0.05 0.002", f"10 {y} 0.002", "hazard_y")

# --- reactor hall equipment (visual only) ---
RX, RY = 12.0, 0.0
tank(RX, RY, 2.4, 6.0, "reactor", "Reactor vessel")
vis("cylinder", f"{2.7} 0.35", f"{RX} {RY} 6.05", "concrete_dark")                      # head
for k in range(8):
    a = k * math.pi / 4; vis("cylinder", "0.12 0.5", f"{RX+1.7*math.cos(a)} {RY+1.7*math.sin(a)} 6.7", "steel")   # drive housings
vis("cylinder", "3.4 0.25", f"{RX} {RY} 0.25", "concrete_dark")                          # bio-shield base
tank(7.0, 6.0, 1.25, 6.5, "pipe_blue", "Steam generator A"); tank(7.0, -6.0, 1.25, 6.5, "pipe_blue", "Steam generator B")
tank(15.5, 5.5, 0.9, 6.0, "pipe_red", "Pressurizer")
for (px, py) in ((9.6, 3.4), (9.6, -3.4)):
    vis("cylinder", "0.6 1.1", f"{px} {py} 1.1", "steel"); shapes.append({"t": "circle", "x": px, "y": py, "r": 0.6, "name": "Coolant pump"})
# turbine (axis along y) and generator
vis("cylinder", "1.3 3.6", "19.2 -3.2 1.6", "steel", 'euler="1.5708 0 0"'); vis("cylinder", "1.1 1.5", "19.2 2.3 1.4", "pipe_yellow", 'euler="1.5708 0 0"')
shapes.append({"t": "rect", "x0": 17.8, "x1": 20.6, "y0": -6.9, "y1": 3.9, "name": "Turbine hall"})
# primary loop pipes (hot legs to the steam generators, cold legs back through the pumps)
pipe((10.1, 1.2, 2.4), (8.2, 4.9, 2.4), 0.32, "pipe_red"); pipe((10.1, -1.2, 2.4), (8.2, -4.9, 2.4), 0.32, "pipe_red")
pipe((7.8, 4.4, 0.8), (9.6, 3.4, 0.8), 0.28, "pipe_blue"); pipe((9.6, 3.4, 0.8), (10.4, 1.4, 0.8), 0.28, "pipe_blue")
pipe((7.8, -4.4, 0.8), (9.6, -3.4, 0.8), 0.28, "pipe_blue"); pipe((9.6, -3.4, 0.8), (10.4, -1.4, 0.8), 0.28, "pipe_blue")
pipe((13.6, 1.4, 4.5), (15.5, 4.9, 4.5), 0.25, "pipe_red")                                  # surge line to the pressurizer
# secondary steam lines to the turbine, with valve wheels
pipe((7.0, 6.0, 6.2), (7.0, 8.2, 6.2), 0.3, "pipe_yellow"); pipe((7.0, 8.2, 6.2), (18.8, 8.2, 6.2), 0.3, "pipe_yellow"); pipe((18.8, 8.2, 6.2), (18.8, 3.0, 3.2), 0.3, "pipe_yellow")
pipe((7.0, -6.0, 6.2), (7.0, -8.4, 6.2), 0.3, "pipe_yellow"); pipe((7.0, -8.4, 6.2), (18.8, -8.4, 6.2), 0.3, "pipe_yellow"); pipe((18.8, -8.4, 6.2), (18.8, -6.6, 3.0), 0.3, "pipe_yellow")
for vx, vy, vz in ((12.0, 8.2, 6.2), (14.0, -8.4, 6.2), (4.4, 6.9, 1.4), (14.0, -6.3, 1.2)):
    vis("cylinder", "0.28 0.03", f"{vx} {vy} {vz+0.4}", "hazard_y"); vis("cylinder", "0.05 0.4", f"{vx} {vy} {vz}", "steel")
# catwalk and railings along the south side
vis("box", "5.5 0.6 0.06", "12 -7.6 2.6", "steel")
for i in range(12): vis("cylinder", "0.03 0.5", f"{6.7 + i*1.0} -7.0 3.1", "hazard_y")
# --- radiation shield walls (visual here, occluders in the radiation model) ---
SHIELDS = [(4.8, 5.3, -3.6, -1.0), (13.4, 13.9, -8.4, -4.4), (9.2, 9.7, 6.0, 8.8)]
for x0, x1, y0, y1 in SHIELDS:
    vis("box", f"{(x1-x0)/2} {(y1-y0)/2} 1.6", f"{(x0+x1)/2} {(y0+y1)/2} 1.6", "concrete"); vis("box", f"{(x1-x0)/2+0.02} {(y1-y0)/2+0.02} 0.1", f"{(x0+x1)/2} {(y0+y1)/2} 3.25", "hazard_y")
    shapes.append({"t": "rect", "x0": x0, "x1": x1, "y0": y0, "y1": y1, "name": "Concrete shield"})
# --- control terminal where the robot reports back ---
TERM = (1.0, -5.0)
vis("box", "0.9 0.45 0.45", f"{TERM[0]} {TERM[1]} 0.45", "steel"); vis("box", "0.8 0.03 0.4", f"{TERM[0]} {TERM[1]+0.4} 1.15", "screen", 'euler="-0.3 0 0"')
vis("box", "0.5 0.25 0.03", f"{TERM[0]} {TERM[1]-0.1} 0.92", "panel"); vis("cylinder", "0.5 0.01", f"{TERM[0]} {TERM[1]} 0.005", "terminal_pad")
vis("cylinder", "0.05 0.9", f"{TERM[0]-0.8} {TERM[1]+0.5} 0.9", "beacon_pole"); vis("sphere", "0.1", f"{TERM[0]-0.8} {TERM[1]+0.5} 1.9", "beacon")
shapes.append({"t": "rect", "x0": TERM[0]-0.9, "x1": TERM[0]+0.9, "y0": TERM[1]-0.45, "y1": TERM[1]+0.45, "name": "Control terminal"})
# warning signs on the east wall, pylons around the entry
for sy in (-5, 0, 5):
    vis("cylinder", "0.45 0.02", f"{X1-0.05} {sy} 2.6", "hazard_y", 'euler="0 1.5708 0"'); vis("cylinder", "0.12 0.03", f"{X1-0.08} {sy} 2.6", "sign_k", 'euler="0 1.5708 0"')
for px, py in ((2.5, 2.5), (2.5, -2.5), (3.5, 3.0), (3.5, -3.0)):
    vis("cylinder", "0.12 0.35", f"{px} {py} 0.35", "hazard_y"); vis("cylinder", "0.125 0.1", f"{px} {py} 0.5", "hazard_k")
# Leak candidates: three of these six are radioactive each run (seeded); all of them vent steam.
CANDIDATES = [
    {"id": "L1", "x": 7.2, "y": 3.9, "z": 1.0, "label": "Steam generator A hot-leg flange"},
    {"id": "L2", "x": 10.4, "y": -3.9, "z": 0.9, "label": "Coolant pump B seal"},
    {"id": "L3", "x": 16.3, "y": 2.4, "z": 1.0, "label": "Pressurizer relief line"},
    {"id": "L4", "x": 14.0, "y": -6.3, "z": 1.2, "label": "Steam valve V-17"},
    {"id": "L5", "x": 4.4, "y": 6.9, "z": 1.4, "label": "Feedwater header"},
    {"id": "L6", "x": 18.4, "y": 5.4, "z": 1.0, "label": "Turbine drain"},
]
for c in CANDIDATES: vis("sphere", "0.07", f"{c['x']} {c['y']} {c['z']}", "pipe_red")        # a small coupling at each point

materials = {
    "floor": "0.2 0.21 0.23 1", "reactor": "0.72 0.74 0.78 1", "wall": "0.55 0.57 0.6 1", "concrete": "0.62 0.62 0.6 1", "concrete_dark": "0.38 0.39 0.4 1",
    "steel": "0.62 0.65 0.7 1", "pipe_red": "0.8 0.2 0.15 1", "pipe_blue": "0.2 0.4 0.75 1", "pipe_yellow": "0.9 0.75 0.15 1",
    "hazard_y": "1 0.8 0.05 1", "hazard_k": "0.07 0.07 0.07 1", "screen": "0.2 1 0.5 1", "panel": "0.12 0.14 0.18 1",
    "terminal_pad": "0.2 0.9 0.5 0.55", "beacon": "1 0.5 0.1 1", "beacon_pole": "0.25 0.25 0.28 1", "sign_k": "0.05 0.05 0.05 1",
}
mat_xml = "\n".join(f'    <material name="{n}" rgba="{c}"' + (' emission="0.9"' if n in ("screen", "beacon", "terminal_pad") else "") + "/>" for n, c in materials.items())
xml = f'''<mujoco model="g1 nuclear plant">
  <include file="g1_with_hands.xml"/>
  <visual>
    <headlight diffuse="0.55 0.55 0.55" ambient="0.3 0.3 0.33" specular="0.3 0.3 0.3"/>
    <global azimuth="140" elevation="-20"/>
  </visual>
  <asset>
{mat_xml}
  </asset>
  <worldbody>
    <light pos="6 0 9" dir="0.2 0 -1" diffuse="0.5 0.48 0.45" castshadow="false"/>
    <light pos="16 4 9" dir="-0.1 -0.1 -1" diffuse="0.35 0.35 0.4" castshadow="false"/>
    <geom name="floor" type="plane" size="30 20 0.05" pos="9 0 0" material="floor" contype="1" conaffinity="1" friction="1 0.005 0.0001"/>
{chr(10).join(g)}
  </worldbody>
</mujoco>
'''
(OUT / "plant.xml").write_text(xml)
mission = {
    "bounds": [X0, X1, Y0, Y1], "start": [0.0, 0.0], "terminal": list(TERM), "candidates": CANDIDATES, "shields": [list(s) for s in SHIELDS],
    "reactor": [RX, RY, 2.4], "shapes": shapes, "budget_gy": 10.0, "report_radius": 2.2, "tag_radius": 3.5,
}
(OUT / "plant_mission.json").write_text(json.dumps(mission))
print("wrote plant.xml", len(g), "geoms")
