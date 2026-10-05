"""Builds the bimanual Franka stacking cell (web/assets/scenes/franka_stack/stack.xml)
from MuJoCo Menagerie's Panda: two prefixed copies of the arm on a table plus
three cubes. Stacking task logic follows robosuite's Stack environment."""
import copy, shutil, sys
import xml.etree.ElementTree as ET
from pathlib import Path

SRC = Path(sys.argv[1])            # menagerie franka_emika_panda folder
OUT = Path("web/assets/scenes/franka_stack")
OUT.mkdir(parents=True, exist_ok=True)
shutil.copytree(SRC / "assets", OUT / "assets", dirs_exist_ok=True)
shutil.copy(SRC / "LICENSE", OUT / "LICENSE_panda")

panda = ET.parse(SRC / "panda.xml").getroot()
root = ET.Element("mujoco", model="bimanual_franka_stack")
ET.SubElement(root, "compiler", angle="radian", meshdir="assets", autolimits="true")
ET.SubElement(root, "option", integrator="implicitfast", timestep="0.002")
ET.SubElement(root, "statistic", center="0 0 0.6", extent="1.4")
vis = ET.SubElement(root, "visual")
ET.SubElement(vis, "headlight", ambient="0.4 0.4 0.4", diffuse="0.6 0.6 0.6", specular="0.1 0.1 0.1")
ET.SubElement(vis, "global", azimuth="90", elevation="-25")
root.append(copy.deepcopy(panda.find("default")))

asset = copy.deepcopy(panda.find("asset"))
for n, rgba, extra in [
    ("table_top", "0.62 0.5 0.36 1", {}), ("table_leg", "0.25 0.25 0.28 1", {}),
    ("cube_red", "0.9 0.2 0.2 1", {}), ("cube_green", "0.2 0.75 0.3 1", {}), ("cube_blue", "0.2 0.4 0.9 1", {}),
    ("floor", "0.78 0.8 0.82 1", {"reflectance": "0.1"}),
]:
    ET.SubElement(asset, "material", name=n, rgba=rgba, **extra)
root.append(asset)

def prefixed(elem, pre):
    e = copy.deepcopy(elem)
    for x in e.iter():
        for a in ("name", "joint", "joint1", "joint2", "tendon", "body1", "body2"):
            if a in x.attrib:
                x.set(a, pre + x.attrib[a])
    return e

wb = ET.SubElement(root, "worldbody")
ET.SubElement(wb, "light", name="key", pos="0 -0.4 2.2", dir="0 0.2 -1", diffuse="0.7 0.7 0.7", castshadow="true")
ET.SubElement(wb, "geom", name="floor", type="plane", size="4 4 0.05", material="floor")
table = ET.SubElement(wb, "body", name="table", pos="0 0 0")
ET.SubElement(table, "geom", name="table_top", type="box", size="0.75 0.5 0.02", pos="0 0 0.38", material="table_top", friction="1 0.005 0.0001")
for i, (x, y) in enumerate([(-0.68, -0.43), (0.68, -0.43), (-0.68, 0.43), (0.68, 0.43)]):
    ET.SubElement(table, "geom", name=f"leg{i}", type="box", size="0.03 0.03 0.18", pos=f"{x} {y} 0.18", material="table_leg")

# Two arms side by side at the table's near edge, both facing +y.
for pre, x in (("l_", -0.38), ("r_", 0.38)):
    base = prefixed(panda.find("worldbody").find("body"), pre)
    base.set("pos", f"{x} -0.42 0.4")
    base.set("quat", "0.7071068 0 0 0.7071068")
    for b in base.iter("body"):
        if b.get("name") == pre + "hand":
            ET.SubElement(b, "site", name=pre + "grip", pos="0 0 0.1034", size="0.006", rgba="1 0.4 0 0.8")
    wb.append(base)

cubes = [("cubeA", "cube_red", (-0.12, 0.12)), ("cubeB", "cube_green", (0.12, 0.12)), ("cubeC", "cube_blue", (0.0, 0.30))]
for name, mat, (x, y) in cubes:
    b = ET.SubElement(wb, "body", name=name, pos=f"{x} {y} 0.43")
    ET.SubElement(b, "freejoint", name=name + "_joint")
    ET.SubElement(b, "geom", name=name + "_geom", type="box", size="0.025 0.025 0.025", mass="0.08",
                  material=mat, friction="1.2 0.01 0.001", condim="4", solimp="0.95 0.99 0.001", solref="0.004 1")

for tag in ("tendon", "equality", "contact"):
    sec = ET.SubElement(root, tag)
    for pre in ("l_", "r_"):
        for child in panda.find(tag):
            sec.append(prefixed(child, pre))
act = ET.SubElement(root, "actuator")
for pre in ("l_", "r_"):
    for child in panda.find("actuator"):
        act.append(prefixed(child, pre))

home_q = "0 0 0 -1.57079 0 1.57079 -0.7853 0.04 0.04"
home_c = "0 0 0 -1.57079 0 1.57079 -0.7853 255"
cube_q = " ".join(f"{x} {y} 0.43 1 0 0 0" for _, _, (x, y) in cubes)
kf = ET.SubElement(root, "keyframe")
ET.SubElement(kf, "key", name="home", qpos=f"{home_q} {home_q} {cube_q}", ctrl=f"{home_c} {home_c}")

ET.indent(root)
ET.ElementTree(root).write(OUT / "stack.xml")
print("wrote", OUT / "stack.xml")
