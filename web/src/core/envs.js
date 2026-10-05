// The environment registry: one entry per playable/benchmarked robot task.
import { G1Controller } from "../robots/g1/controller.js";
import { FrankaController } from "../robots/franka/controller.js";
import { MarsController } from "../robots/husky/controller.js";

export const ENVS = [
  {
    id: "g1", name: "Unitree G1", kind: "Humanoid", task: "Nuclear plant leak",
    story: "g1",
    mission: "Reactor 3 is leaking. Walk the plant with your dosimeter, find the radioactive leaks among the steaming pipes, flag them and report back before the radiation kills your electronics.",
    blurb: "Pretrained walking policy. Hunt radiation with a Geiger counter.",
    dir: "g1_plant", scene: "g1_plant/plant.xml", timeLimit: 300,
    make: () => new G1Controller(),
    help: "<b>W A S D</b> walk / turn &nbsp;·&nbsp; <b>F</b> plant a flag on a leak &nbsp;·&nbsp; <b>G</b> undo flag &nbsp;·&nbsp; <b>Enter</b> report (at the green terminal pad) &nbsp;·&nbsp; <b>Space</b> jump &nbsp;·&nbsp; <b>C</b> camera &nbsp;·&nbsp; <b>M</b> map &nbsp;·&nbsp; <b>N</b> mute",
    camera: { pos: [-2.4, 2.1, 1.7], target: [3.5, 0.9, -0.2] },
    camModes: "fpv", sky: [0.07, 0.09, 0.13], fog: [0.07, 0.09, 0.13, 22, 48], ambient: 0.9,
  },
  {
    id: "franka", name: "Bimanual Franka", kind: "Manipulation", task: "Biohazard containment",
    story: "franka",
    mission: "A virus sample has leaked in a BSL-4 lab. Relay four sample tubes from the rack (left arm only) through the airlock to the biohazard case (right arm only), then seal it before the purge.",
    blurb: "Two Franka Panda arms, real contact friction, a dangerous handover.",
    dir: "franka_lab", scene: "franka_lab/biolab.xml", timeLimit: 240, countdown: true,
    make: () => new FrankaController(),
    help: "<b>Tab</b> switch arm &nbsp;·&nbsp; <b>W A S D</b> move gripper &nbsp;·&nbsp; <b>R / F</b> up / down &nbsp;·&nbsp; <b>Q / E</b> rotate wrist &nbsp;·&nbsp; <b>Space</b> open / close gripper &nbsp;·&nbsp; <b>C</b> camera view",
    camera: { pos: [0, 1.5, 2.0], target: [0, 0.5, -0.1] },
    camModes: [{ pos: [0, 1.5, 2.0], target: [0, 0.5, -0.1] }, { pos: [0, 2.0, 0.3], target: [0, 0.4, -0.14] }, { pos: [1.6, 1.0, 0.9], target: [0, 0.5, -0.1] }],
    sky: [0.12, 0.14, 0.2],
  },
  {
    id: "rover", name: "Husky on Mars", kind: "Rover", task: "Sample return", story: "rover",
    mission: "Scientists may have found life on Mars. Reach the survey beacon, collect the sample, and bring it back to the lander using only the rover's own SLAM map, before the battery dies.",
    blurb: "Clearpath Husky off-roading across craters and boulders on Mars.",
    dir: "rover_mars", scene: "rover_mars/mars.xml", timeLimit: 420,
    make: () => new MarsController(),
    help: "<b>W S</b> drive &nbsp;·&nbsp; <b>A D</b> steer &nbsp;·&nbsp; <b>Space</b> brake &nbsp;·&nbsp; <b>Hold E</b> at the sample site to collect &nbsp;·&nbsp; <b>C</b> camera &nbsp;·&nbsp; <b>M</b> enlarge map &nbsp;·&nbsp; <b>T</b> true path",
    camera: { pos: [-18, 6, 18], target: [-20, 1, 24] },
    camModes: "chase", sky: [0.80, 0.55, 0.40], fog: [0.80, 0.55, 0.40, 40, 120], ambient: 1.6, sun: 2.4,
  },
];
export const envById = (id) => ENVS.find((e) => e.id === id);
