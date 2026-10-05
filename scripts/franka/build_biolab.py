"""Builds the BSL-4 biohazard lab (web/assets/scenes/franka_lab/biolab.xml).

Two Franka Panda arms (MuJoCo Menagerie model, Apache-2.0) stand at a lab bench. A rack of
sample tubes sits at the far LEFT where only the left arm can reach, a biohazard transport case at
the far RIGHT where only the right arm can reach, so each tube has to be handed over in the middle.
A SEAL button on the case finishes the job.

usage: python scripts/franka/build_biolab.py [path/to/menagerie/franka_emika_panda]
(the arm meshes are already in web/assets/scenes/franka_lab/assets; pass the path to refresh them)
"""
import copy, math, shutil, sys
import xml.etree.ElementTree as ET
from pathlib import Path

OUT = Path("web/assets/scenes/franka_lab")
OUT.mkdir(parents=True, exist_ok=True)
if len(sys.argv) > 1:
    src = Path(sys.argv[1]); shutil.copytree(src / "assets", OUT / "assets", dirs_exist_ok=True); shutil.copy(src / "LICENSE", OUT / "LICENSE_panda")
panda = ET.parse("scripts/franka/data/panda.xml").getroot()

TABLE_Z = 0.40
TUBE_R, TUBE_HALF = 0.016, 0.06
TUBE_YS = [0.0, 0.08, 0.16, 0.24]; RACK_X = -0.60
CASE = (0.60, 0.16)           # biohazard case centre (x, y)
BUTTON = (0.60, -0.06)
AIRLOCK = (0.0, 0.14)         # pass-through tray in the middle, reachable by both arms

root = ET.Element("mujoco", model="biohazard_lab")
ET.SubElement(root, "compiler", angle="radian", meshdir="assets", autolimits="true")
ET.SubElement(root, "option", integrator="implicitfast", timestep="0.002")
ET.SubElement(root, "statistic", center="0 0 0.6", extent="1.6")
vis = ET.SubElement(root, "visual")
ET.SubElement(vis, "headlight", ambient="0.35 0.35 0.38", diffuse="0.55 0.55 0.55", specular="0.1 0.1 0.1")
root.append(copy.deepcopy(panda.find("default")))
asset = copy.deepcopy(panda.find("asset"))
MATS = {
    "floor": "0.62 0.64 0.66 1", "wall": "0.82 0.86 0.88 1", "bench": "0.78 0.8 0.82 1", "bench_leg": "0.28 0.3 0.34 1",
    "hazard_y": "1 0.8 0.05 1", "hazard_k": "0.06 0.06 0.06 1", "case_red": "0.85 0.12 0.1 1", "case_in": "0.25 0.05 0.05 1",
    "tube": "0.9 0.95 1 0.35", "cap_red": "0.95 0.12 0.1 1", "cap_green": "0.15 0.8 0.3 1", "cap_blue": "0.2 0.4 0.95 1", "cap_yellow": "1 0.8 0.1 1",
    "virus": "0.3 1 0.35 0.8", "rack": "0.2 0.24 0.3 1", "button": "0.9 0.05 0.05 1", "button_base": "0.95 0.8 0.1 1", "sign_k": "0.05 0.05 0.05 1",
    "beacon": "1 0.1 0.05 1", "spill": "0.35 1 0.4 0.45", "glass": "0.7 0.85 0.95 0.18", "panel": "0.15 0.2 0.25 1",
}
for n, rgba in MATS.items():
    kw = {"emission": "0.9"} if n in ("virus", "beacon", "button") else {}
    ET.SubElement(asset, "material", name=n, rgba=rgba, **kw)
root.append(asset)

def prefixed(elem, pre):
    e = copy.deepcopy(elem)
    for x in e.iter():
        for a in ("name", "joint", "joint1", "joint2", "tendon", "body1", "body2"):
            if a in x.attrib: x.set(a, pre + x.attrib[a])
    return e

wb = ET.SubElement(root, "worldbody")
ET.SubElement(wb, "light", name="key", pos="0 -0.5 2.4", dir="0 0.2 -1", diffuse="0.8 0.8 0.8", castshadow="true")
ET.SubElement(wb, "geom", name="floor", type="plane", size="4 4 0.05", material="floor")
def G(parent, **kw): return ET.SubElement(parent, "geom", **{k: str(v) for k, v in kw.items()})

# --- room: back wall, side panels, hazard tape on the floor ---
G(wb, type="box", size="2.2 0.05 1.3", pos="0 1.25 1.3", material="wall", contype=0, conaffinity=0)
G(wb, type="box", size="0.05 1.4 1.3", pos="-2.2 -0.1 1.3", material="wall", contype=0, conaffinity=0)
G(wb, type="box", size="0.05 1.4 1.3", pos="2.2 -0.1 1.3", material="wall", contype=0, conaffinity=0)
for i in range(18):
    x = -1.7 + i * 0.2
    G(wb, type="box", size="0.1 0.5 0.003", pos=f"{x} 0.0 0.003", material="hazard_y" if i % 2 == 0 else "hazard_k", contype=0, conaffinity=0, euler="0 0 0.5")
# biohazard trefoil sign on the back wall (axis along y)
def disc(r, x, z, mat, y=1.19, h=0.01): G(wb, type="cylinder", size=f"{r} {h}", pos=f"{x} {y} {z}", euler="1.5708 0 0", material=mat, contype=0, conaffinity=0)
SX, SZ = 0.0, 1.45
disc(0.62, SX, SZ, "hazard_y", 1.2, 0.012)
for k in range(3):
    a = math.radians(90 + 120 * k)
    cx, cz = SX + 0.27 * math.cos(a), SZ + 0.27 * math.sin(a)
    disc(0.22, cx, cz, "sign_k", 1.185, 0.012); disc(0.16, cx, cz, "hazard_y", 1.18, 0.012)
disc(0.1, SX, SZ, "sign_k", 1.185, 0.012); disc(0.05, SX, SZ, "hazard_y", 1.18, 0.012)
# glass fume-hood panels and warning beacon
G(wb, type="box", size="0.9 0.02 0.45", pos="-1.2 0.9 1.0", material="glass", contype=0, conaffinity=0)
G(wb, type="box", size="0.9 0.02 0.45", pos="1.2 0.9 1.0", material="glass", contype=0, conaffinity=0)
beacon = ET.SubElement(wb, "body", name="beacon", pos="-1.0 0.55 0.0")
G(beacon, type="cylinder", size="0.025 0.55", pos="0 0 0.55", material="bench_leg", contype=0, conaffinity=0)
G(beacon, type="sphere", size="0.07", pos="0 0 1.15", material="beacon", contype=0, conaffinity=0)

# --- bench ---
bench = ET.SubElement(wb, "body", name="bench", pos="0 0 0")
G(bench, name="bench_top", type="box", size="0.95 0.5 0.02", pos=f"0 0 {TABLE_Z - 0.02}", material="bench", friction="1 0.005 0.0001")
for i, (x, y) in enumerate([(-0.88, -0.43), (0.88, -0.43), (-0.88, 0.43), (0.88, 0.43)]):
    G(bench, type="box", size="0.03 0.03 0.18", pos=f"{x} {y} 0.18", material="bench_leg")

# --- two Panda arms, mirrored, both facing +y ---
for pre, x in (("l_", -0.38), ("r_", 0.38)):
    base = prefixed(panda.find("worldbody").find("body"), pre)
    base.set("pos", f"{x} -0.42 {TABLE_Z}"); base.set("quat", "0.7071068 0 0 0.7071068")
    for b in base.iter("body"):
        if b.get("name") == pre + "hand": ET.SubElement(b, "site", name=pre + "grip", pos="0 0 0.1034", size="0.006", rgba="1 0.4 0 0.8")
    wb.append(base)

# --- sample rack (left) with four tubes lying in rails ---
rack = ET.SubElement(wb, "body", name="rack", pos=f"{RACK_X} 0.12 {TABLE_Z}")
G(rack, type="box", size="0.09 0.19 0.006", pos="0 0 0.006", material="rack")
for y in TUBE_YS:
    for s in (-1, 1):
        G(rack, type="box", size="0.07 0.003 0.001", pos=f"0 {y - 0.12 + s * 0.022} 0.013", material="rack")
spill = ET.SubElement(wb, "body", name="spill", pos="-0.78 0.34 0")
G(spill, type="cylinder", size="0.06 0.0015", pos=f"0 0 {TABLE_Z + 0.0015}", material="spill", contype=0, conaffinity=0)
tubes = []
caps = ["cap_red", "cap_red", "cap_red", "cap_green"]
for i, y in enumerate(TUBE_YS):
    name = f"tube{i}"
    b = ET.SubElement(wb, "body", name=name, pos=f"{RACK_X} {y} {TABLE_Z + 0.006 + TUBE_R + 0.0006}")
    ET.SubElement(b, "freejoint", name=name + "_joint")
    G(b, name=name + "_geom", type="cylinder", size=f"{TUBE_R} {TUBE_HALF}", euler="0 1.5708 0", material="tube", mass="0.015", friction="1.6 0.15 0.01", condim="6", solimp="0.97 0.995 0.0005", solref="0.003 1")
    G(b, type="cylinder", size=f"{TUBE_R - 0.003} {TUBE_HALF - 0.012}", pos=f"{-0.006} 0 0", euler="0 1.5708 0", material="virus", contype=0, conaffinity=0, mass="0")
    G(b, name=name + "_cap", type="cylinder", size=f"{TUBE_R + 0.0015} 0.009", pos=f"{TUBE_HALF + 0.003} 0 0", euler="0 1.5708 0", material=caps[i], mass="0.004", friction="1.6 0.05 0.005")
    tubes.append((name, y))

# --- airlock pass-through tray (centre) ---
ax, ay = AIRLOCK
lock = ET.SubElement(wb, "body", name="airlock", pos=f"{ax} {ay} {TABLE_Z}")
G(lock, type="box", size="0.11 0.10 0.005", pos="0 0 0.005", material="panel")
G(lock, type="box", size="0.11 0.004 0.006", pos="0 0.096 0.011", material="hazard_y"); G(lock, type="box", size="0.11 0.004 0.006", pos="0 -0.096 0.011", material="hazard_y")
G(lock, type="box", size="0.004 0.10 0.006", pos="0.106 0 0.011", material="hazard_y"); G(lock, type="box", size="0.004 0.10 0.006", pos="-0.106 0 0.011", material="hazard_y")
G(lock, type="box", size="0.11 0.10 0.0012", pos="0 0 0.0105", material="virus", contype=0, conaffinity=0)

# --- biohazard case (right) ---
cx, cy = CASE
case = ET.SubElement(wb, "body", name="case", pos=f"{cx} {cy} {TABLE_Z}")
G(case, type="box", size="0.115 0.075 0.006", pos="0 0 0.006", material="case_red")
G(case, type="box", size="0.115 0.006 0.045", pos="0 0.069 0.045", material="case_red"); G(case, type="box", size="0.115 0.006 0.045", pos="0 -0.069 0.045", material="case_red")
G(case, type="box", size="0.006 0.075 0.045", pos="0.109 0 0.045", material="case_red"); G(case, type="box", size="0.006 0.075 0.045", pos="-0.109 0 0.045", material="case_red")
G(case, type="box", size="0.10 0.062 0.003", pos="0 0 0.0135", material="case_in", contype=0, conaffinity=0)
G(case, type="cylinder", size="0.032 0.002", pos="0 0.0 0.0135", material="hazard_y", contype=0, conaffinity=0)
# --- SEAL button: a spring-loaded plunger ---
bx, by = BUTTON
base = ET.SubElement(wb, "body", name="button_base", pos=f"{bx} {by} {TABLE_Z}")
G(base, type="cylinder", size="0.036 0.008", pos="0 0 0.008", material="button_base")
btn = ET.SubElement(wb, "body", name="seal_button", pos=f"{bx} {by} {TABLE_Z + 0.028}")
ET.SubElement(btn, "joint", name="seal_slide", type="slide", axis="0 0 1", range="-0.022 0", stiffness="250", damping="8")
G(btn, name="seal_geom", type="cylinder", size="0.026 0.012", material="button", mass="0.05")

for tag in ("tendon", "equality", "contact"):
    sec = ET.SubElement(root, tag)
    for pre in ("l_", "r_"):
        for child in panda.find(tag): sec.append(prefixed(child, pre))
    if tag == "contact": ET.SubElement(sec, "exclude", body1="button_base", body2="seal_button")
act = ET.SubElement(root, "actuator")
for pre in ("l_", "r_"):
    for child in panda.find("actuator"):
        c = prefixed(child, pre)
        if c.get("tendon"):                       # stronger grip so off-centre tube grasps hold: ~15 N per finger
            c.set("gainprm", "0.1882 0 0"); c.set("biasprm", "0 -1200 -120")
        act.append(c)

home_q = "0 0 0 -1.57079 0 1.57079 -0.7853 0.04 0.04"; home_c = "0 0 0 -1.57079 0 1.57079 -0.7853 255"
tube_q = " ".join(f"{RACK_X} {y} {TABLE_Z + 0.006 + TUBE_R + 0.0006} 1 0 0 0" for _, y in tubes)
kf = ET.SubElement(root, "keyframe")
ET.SubElement(kf, "key", name="home", qpos=f"{home_q} {home_q} {tube_q} 0", ctrl=f"{home_c} {home_c}")
ET.indent(root)
ET.ElementTree(root).write(OUT / "biolab.xml")
print("wrote", OUT / "biolab.xml")
