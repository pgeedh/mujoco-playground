// The environment registry: one entry per playable/benchmarked robot task.
import { G1Controller } from "./g1Control.js";
import { FrankaController } from "./frankaControl.js";
import { MarsController } from "./marsControl.js";

export const ENVS = [
  {
    id: "g1", name: "Unitree G1", kind: "Humanoid", task: "Dishwasher loading",
    mission: "Carry the 3 plates from the counter to the dishwasher down the path. Falls cost points.",
    blurb: "Pretrained walking policy. Grab, carry and jump.",
    dir: "g1_park", scene: "g1_park/world.xml", timeLimit: 180,
    make: () => new G1Controller(),
    help: "<b>W A S D</b> walk/turn &nbsp;·&nbsp; <b>Space</b> jump &nbsp;·&nbsp; <b>Q / E</b> grab or release with right / left hand &nbsp;·&nbsp; <b>C</b> first/third-person camera",
    camera: { pos: [-2.2, 1.5, 1.8], target: [0, 0.7, 0] },
    camModes: "fpv", sky: [0.15, 0.25, 0.35],
  },
  {
    id: "franka", name: "Bimanual Franka", kind: "Manipulation", task: "Cube stacking",
    mission: "Stack the red cube on the green cube, then the blue cube on top. Two Panda arms, real contact friction.",
    blurb: "Two Franka Panda arms on a table. Grasp and stack with real friction.",
    dir: "franka_stack", scene: "franka_stack/stack.xml", timeLimit: 180,
    make: () => new FrankaController(),
    help: "<b>Tab</b> switch arm &nbsp;·&nbsp; <b>W A S D</b> move gripper (away / left / toward / right) &nbsp;·&nbsp; <b>R / F</b> up / down &nbsp;·&nbsp; <b>Q / E</b> rotate wrist &nbsp;·&nbsp; <b>Space</b> open / close gripper &nbsp;·&nbsp; <b>C</b> camera view",
    camera: { pos: [0, 1.6, 2.0], target: [0, 0.5, -0.15] },
    camModes: [{ pos: [0, 1.6, 2.0], target: [0, 0.5, -0.15] }, { pos: [0, 1.9, 0.2], target: [0, 0.4, -0.12] }, { pos: [1.3, 0.9, 0.6], target: [0, 0.5, -0.1] }],
    sky: [0.16, 0.2, 0.28],
  },
  {
    id: "rover", name: "Husky on Mars", kind: "Rover", task: "Sample return",
    mission: "Follow the map: reach the blue survey beacon, collect the sample at the green site, then return to the lander.",
    blurb: "Clearpath Husky off-roading across craters and boulders on Mars.",
    dir: "rover_mars", scene: "rover_mars/mars.xml", timeLimit: 240,
    make: () => new MarsController(),
    help: "<b>W S</b> drive &nbsp;·&nbsp; <b>A D</b> steer &nbsp;·&nbsp; <b>Space</b> brake &nbsp;·&nbsp; <b>Hold E</b> at the sample site to collect &nbsp;·&nbsp; <b>C</b> chase / free camera &nbsp;·&nbsp; <b>M</b> enlarge map",
    camera: { pos: [-18, 6, 18], target: [-20, 1, 24] },
    camModes: "chase", sky: [0.80, 0.55, 0.40], fog: [0.80, 0.55, 0.40, 40, 120], ambient: 1.6, sun: 2.4,
  },
];
export const envById = (id) => ENVS.find((e) => e.id === id);
