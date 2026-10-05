"""Builds the Mars sample-return scene (web/assets/scenes/rover_mars/mars.xml):
procedural heightfield terrain (craters, dunes, boulders), a lander, mission
beacons and a Clearpath Husky (BSD-3 meshes from github.com/husky/husky)."""
import json
from pathlib import Path
import numpy as np
from PIL import Image

OUT = Path("web/assets/scenes/rover_mars")
N, R, ZMAX = 193, 30.0, 3.0            # grid size, half-extent (m), max elevation (m)
rng = np.random.default_rng(7)
xs = np.linspace(-R, R, N)
X, Y = np.meshgrid(xs, xs)               # row index -> y, col index -> x

# ---- terrain ----------------------------------------------------------------
H = np.full((N, N), 1.4)
for _ in range(7):                       # rolling low-frequency relief
    kx, ky = rng.uniform(0.05, 0.22, 2); ph = rng.uniform(0, 6.28, 2)
    H += 0.22 * np.sin(kx * X + ph[0]) * np.cos(ky * Y + ph[1])
dune_dir = np.array([0.8, 0.6])
H += 0.12 * np.sin((X * dune_dir[0] + Y * dune_dir[1]) * 0.9 + 0.5 * np.sin(Y * 0.15))   # dune ripples
CRATERS = [(-4, -2, 8.5, 0.8), (12, -14, 7.0, 0.7), (-16, 12, 7.5, 0.75), (18, 16, 6.0, 0.6), (2, 20, 5.0, 0.45)]
for cx, cy, cr, depth in CRATERS:
    r = np.hypot(X - cx, Y - cy)
    bowl = np.where(r < cr, depth * np.clip(1 - (r / cr) ** 2, 0, None) ** 1.5, 0)
    rim = 0.2 * depth * np.exp(-(((r - cr) / (0.22 * cr)) ** 2))
    H += -bowl + rim

# Fine-scale gravel/ripple roughness so the ride is bumpy (a few cm).
from scipy.ndimage import gaussian_filter
H += 0.045 * gaussian_filter(rng.normal(size=(N, N)), 1.2) / 0.28
H += 0.02 * np.sin(X * 3.1 + 1.3 * np.sin(Y * 2.3)) * np.cos(Y * 2.7)

def flatten(cx, cy, rad, blend=2.5):
    r = np.hypot(X - cx, Y - cy)
    w = np.clip((r - rad) / blend, 0, 1)
    target = H[np.unravel_index(np.argmin(r), r.shape)]
    return w, target
LANDER = (-24.0, -24.0)
SITES = {"alpha": (-7.0, -15.0), "home": LANDER, "sample0": (15.0, 11.0), "sample1": (10.0, -22.0),
         "sample2": (-20.0, 17.0), "sample3": (22.0, -4.0)}
for key, (cx, cy) in SITES.items():
    w, t = flatten(cx, cy, 2.5 if key != "home" else 5.5)
    H = H * w + t * (1 - w)
H = np.clip(H - H.min(), 0, None)
H = H / H.max() * ZMAX
hf = (H / ZMAX * 255).round().astype(np.uint8)
H = hf.astype(float) / 255 * ZMAX        # quantised, matches what MuJoCo loads
Image.fromarray(np.flipud(hf)).save(OUT / "assets" / "heightmap.png")   # MuJoCo hfield row 0 = +y

def height_at(x, y):
    fx = (x + R) / (2 * R) * (N - 1); fy = (y + R) / (2 * R) * (N - 1)
    i0, j0 = int(np.clip(np.floor(fy), 0, N - 2)), int(np.clip(np.floor(fx), 0, N - 2)); ty, tx = fy - i0, fx - j0
    return (H[i0, j0] * (1 - tx) * (1 - ty) + H[i0, j0 + 1] * tx * (1 - ty) + H[i0 + 1, j0] * (1 - tx) * ty + H[i0 + 1, j0 + 1] * tx * ty)

# Visual terrain mesh (MuJoCo hfield geoms are collision-only for the viewer).
with open(OUT / "assets" / "terrain_visual.obj", "w") as f:
    for i in range(N):
        for j in range(N):
            f.write(f"v {xs[j]:.4f} {xs[i]:.4f} {H[i, j]:.4f}\n")
    for i in range(N):
        for j in range(N):
            f.write(f"vt {j / (N - 1):.5f} {i / (N - 1):.5f}\n")          # planar UVs for the ground photo
    for i in range(N - 1):
        for j in range(N - 1):
            a = i * N + j + 1; b = a + 1; c = a + N; d = c + 1
            f.write(f"f {a}/{a} {b}/{b} {d}/{d}\nf {a}/{a} {d}/{d} {c}/{c}\n")

# ---- boulders -----------------------------------------------------------------
keep_clear = [np.array(p) for p in SITES.values()]
rocks = []
while len(rocks) < 70:
    x, y = rng.uniform(-R + 2, R - 2, 2)
    if any(np.hypot(x - p[0], y - p[1]) < 5.5 for p in keep_clear): continue
    s = float(rng.choice([0.18, 0.25, 0.35, 0.5, 0.75], p=[0.3, 0.3, 0.2, 0.15, 0.05]))
    rocks.append((x, y, s, rng.uniform(0.7, 1.3), rng.uniform(0.5, 0.9), rng.uniform(0, 3.14)))
rock_xml = []
for i, (x, y, s, sy, sz, yaw) in enumerate(rocks):
    z = height_at(x, y) + s * sz * 0.35
    rock_xml.append(f'    <geom name="rock{i}" type="ellipsoid" size="{s:.2f} {s*sy:.2f} {s*sz:.2f}" pos="{x:.2f} {y:.2f} {z:.2f}" euler="0 0 {yaw:.2f}" material="rock{i%3}"/>')

# ---- Husky --------------------------------------------------------------------
WB, TR, WZ, WR, WW = 0.512, 0.555, 0.03282, 0.1651, 0.1143
BASE_Z = WR - WZ                          # base_link origin height above ground
wheels = []
for name, sx, sy in (("fl", 1, 1), ("fr", 1, -1), ("rl", -1, 1), ("rr", -1, -1)):
    wheels.append(f'''      <body name="wheel_{name}" pos="{sx*WB/2} {sy*TR/2} {WZ}">
        <joint name="wj_{name}" axis="0 1 0" damping="0.2" armature="0.01"/>
        <geom type="cylinder" size="{WR} {WW/2}" euler="1.5708 0 0" mass="2.64" friction="0.75 0.01 0.001" material="tire" condim="3"/>
        <geom type="mesh" mesh="wheel" contype="0" conaffinity="0" material="tire" mass="0"/>
      </body>''')

START = (-20.0, -24.0)
sz0 = height_at(*START) + BASE_Z + 0.04
LX, LY = LANDER
lz = height_at(LX, LY)
xml = f'''<mujoco model="rover_mars">
  <compiler angle="radian" meshdir="assets" texturedir="assets"/>
  <option timestep="0.004" integrator="implicitfast" gravity="0 0 -3.71"/>
  <statistic center="0 0 1" extent="45"/>
  <visual><headlight ambient="0.45 0.35 0.3" diffuse="0.55 0.45 0.4" specular="0.05 0.05 0.05"/></visual>
  <asset>
    <hfield name="mars" file="heightmap.png" size="{R} {R} {ZMAX} 0.5"/>
    <mesh name="terrain_vis" file="terrain_visual.obj"/>
    <mesh name="base_link" file="base_link.stl"/>
    <mesh name="top_chassis" file="top_chassis.stl"/>
    <mesh name="user_rail" file="user_rail.stl"/>
    <mesh name="top_plate" file="top_plate.stl"/>
    <mesh name="wheel" file="wheel.stl" scale="0.93 1 0.93"/>
    <texture name="mars_ground_tex" type="2d" file="mars_ground.png"/>
    <material name="regolith" texture="mars_ground_tex" texrepeat="34 34" rgba="1 1 1 1"/>
    <material name="rock0" rgba="0.42 0.27 0.2 1"/>
    <material name="rock1" rgba="0.5 0.33 0.24 1"/>
    <material name="rock2" rgba="0.34 0.22 0.17 1"/>
    <material name="husky_yellow" rgba="0.98 0.78 0.1 1"/>
    <material name="husky_black" rgba="0.12 0.12 0.13 1"/>
    <material name="tire" rgba="0.1 0.1 0.1 1"/>
    <material name="steel" rgba="0.7 0.72 0.75 1"/>
    <material name="lander" rgba="0.85 0.85 0.88 1"/>
    <material name="gold" rgba="0.9 0.7 0.2 1"/>
    <material name="beacon_a" rgba="0.2 0.6 1 0.35"/>
    <material name="beacon_s" rgba="0.2 1 0.6 0.4"/>
    <material name="beacon_h" rgba="1 0.85 0.2 0.35"/>
    <material name="sample_mat" rgba="0.3 1 0.9 1" emission="0.8"/>
  </asset>
  <worldbody>
    <light name="sun" pos="-20 -10 40" dir="0.5 0.3 -1" diffuse="0.9 0.8 0.7" castshadow="true"/>
    <geom name="terrain" type="hfield" hfield="mars" friction="0.75 0.01 0.001" group="3"/>
    <geom type="mesh" mesh="terrain_vis" material="regolith" contype="0" conaffinity="0" group="1"/>
{chr(10).join(rock_xml)}
    <body name="lander" pos="{LX} {LY} {lz:.3f}">
      <geom type="box" size="1.1 1.1 0.12" pos="0 0 1.1" material="lander"/>
      <geom type="box" size="0.8 0.8 0.35" pos="0 0 1.45" material="gold"/>
      <geom type="cylinder" size="0.05 0.9" pos="1.0 1.0 0.6" euler="0 0.35 0" material="steel"/>
      <geom type="cylinder" size="0.05 0.9" pos="-1.0 1.0 0.6" euler="0 -0.35 0" material="steel"/>
      <geom type="cylinder" size="0.05 0.9" pos="1.0 -1.0 0.6" euler="0 0.35 0" material="steel"/>
      <geom type="cylinder" size="0.05 0.9" pos="-1.0 -1.0 0.6" euler="0 -0.35 0" material="steel"/>
      <geom type="box" size="0.9 0.04 0.5" pos="0 1.3 1.9" euler="0.5 0 0" material="steel" contype="0" conaffinity="0"/>
    </body>
    <body name="beacon_alpha" mocap="true" pos="{SITES['alpha'][0]} {SITES['alpha'][1]} {height_at(*SITES['alpha']):.3f}">
      <geom type="cylinder" size="0.5 4" pos="0 0 4" material="beacon_a" contype="0" conaffinity="0"/></body>
    <body name="beacon_sample" mocap="true" pos="{SITES['sample0'][0]} {SITES['sample0'][1]} {height_at(*SITES['sample0']):.3f}">
      <geom type="cylinder" size="0.5 4" pos="0 0 4" material="beacon_s" contype="0" conaffinity="0"/></body>
    <body name="beacon_home" mocap="true" pos="{LX+4} {LY} {height_at(LX+4, LY):.3f}">
      <geom type="cylinder" size="0.5 4" pos="0 0 4" material="beacon_h" contype="0" conaffinity="0"/></body>
    <body name="sample" mocap="true" pos="{SITES['sample0'][0]} {SITES['sample0'][1]} {height_at(*SITES['sample0'])+0.2:.3f}">
      <geom type="capsule" size="0.07 0.12" material="sample_mat" contype="0" conaffinity="0"/></body>
    <body name="husky" pos="{START[0]} {START[1]} {sz0:.3f}" quat="1 0 0 0">
      <freejoint name="husky_free"/>
      <inertial mass="46.034" pos="0 0 0.062" diaginertia="0.6022 1.7386 2.0296"/>
      <geom type="mesh" mesh="base_link" material="husky_black" contype="0" conaffinity="0" mass="0"/>
      <geom type="mesh" mesh="top_chassis" material="husky_yellow" contype="0" conaffinity="0" mass="0"/>
      <geom type="mesh" mesh="user_rail" pos="0.0 0 0.2237" material="steel" contype="0" conaffinity="0" mass="0"/>
      <geom type="mesh" mesh="top_plate" pos="0 0 0.2237" material="steel" contype="0" conaffinity="0" mass="0"/>
      <geom type="box" size="0.04 0.285 0.05" pos="0.47 0 0.06" material="husky_black" contype="0" conaffinity="0" mass="0"/>
      <geom type="box" size="0.04 0.285 0.05" pos="-0.47 0 0.06" material="husky_black" contype="0" conaffinity="0" mass="0"/>
      <geom name="chassis_low" type="box" size="0.4937 0.28545 0.0309" pos="0 0 0.0619" mass="0" friction="0.6 0.01 0.001" rgba="0 0 0 0"/>
      <geom name="chassis_top" type="box" size="0.395 0.28545 0.0519" pos="0 0 0.1756" mass="0" friction="0.6 0.01 0.001" rgba="0 0 0 0"/>
      <geom type="box" size="0.05 0.06 0.045" pos="0.5 0 0.04" material="husky_black" contype="0" conaffinity="0" mass="0"/>
      <geom type="cylinder" size="0.035 0.02" pos="0.5 0 0.095" material="steel" contype="0" conaffinity="0" mass="0"/>
      <site name="lidar" pos="0.5 0 0.05" size="0.01"/>
      <site name="bumper" type="box" size="0.52 0.31 0.12" pos="0 0 0.12" rgba="0 0 0 0"/>
      <site name="imu" pos="0.19 0 0.149"/>
{chr(10).join(wheels)}
    </body>
  </worldbody>
  <actuator>
    <velocity name="a_fl" joint="wj_fl" kv="150" ctrlrange="-14 14" forcerange="-400 400"/>
    <velocity name="a_fr" joint="wj_fr" kv="150" ctrlrange="-14 14" forcerange="-400 400"/>
    <velocity name="a_rl" joint="wj_rl" kv="150" ctrlrange="-14 14" forcerange="-400 400"/>
    <velocity name="a_rr" joint="wj_rr" kv="150" ctrlrange="-14 14" forcerange="-400 400"/>
  </actuator>
  <sensor>
    <touch name="bump" site="bumper"/>
  </sensor>
</mujoco>
'''
(OUT / "mars.xml").write_text(xml)
meta = {"half_extent": R, "zmax": ZMAX, "n": N, "heightmap": "assets/heightmap.png", "lander": LANDER, "start": START, "alpha": SITES["alpha"],
        "samples": [SITES[f"sample{i}"] for i in range(4)], "craters": CRATERS, "rocks": [[round(float(x), 2), round(float(y), 2), s, round(float(sy), 3), round(float(sz), 3), round(float(yaw), 3)] for x, y, s, sy, sz, yaw in rocks]}
(OUT / "mission.json").write_text(json.dumps(meta))
print("wrote mars.xml; rocks", len(rocks), "zmax", H.max())
