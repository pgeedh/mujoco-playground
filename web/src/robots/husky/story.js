// Storyboard for the Mars mission: the discovery, the route, the sample, the way home.
import { m2t } from "../../core/briefing.js";

export function huskyStory(ctl) {
  const m = ctl.mission, g = (x, y) => ctl._ground(x, y);
  const sample = m.samples[0];
  return [
    { kicker: "Sol 1,247 · Jezero Crater, Mars", title: "The signal", stamp: "BIOSIGNATURE?",
      text: "An orbiter has found a methane plume and a patch of layered, spotted rock on the crater floor that looks like the fossil of a microbial mat. If it is what it looks like, it is the first evidence of life beyond Earth.",
      cam: { pos: m2t(m.start[0] - 6, m.start[1] - 14, g(m.start[0], m.start[1]) + 14), target: m2t(m.alpha[0], m.alpha[1], g(m.alpha[0], m.alpha[1])) } },
    { kicker: "The lander", title: "One rover, one chance",
      text: "Lander AURORA-1 set down 40 metres from the site. Its only mobile asset is Husky-1: a Clearpath Husky with a front lidar, a gyro and a downward camera for visual odometry. Mars gravity is just 0.38 g, so slopes and boulders behave very differently from Earth.",
      cam: { pos: m2t(m.lander[0] + 6, m.lander[1] - 5, g(m.lander[0], m.lander[1]) + 3.2), target: m2t(m.lander[0], m.lander[1], g(m.lander[0], m.lander[1]) + 1.2) } },
    { kicker: "No GPS on Mars", title: "Navigate with SLAM",
      text: "Husky builds its own map as it drives, matching lidar scans against what it has already seen (SLAM) on top of the orbital image. Wheel odometry slips in the dust, so trust the green SLAM path over the red odometry path. Boulders show up on the map only once the lidar has seen them.",
      cam: { pos: m2t(m.start[0] + 3, m.start[1] - 3, g(m.start[0], m.start[1]) + 2.2), target: m2t(m.alpha[0], m.alpha[1], g(m.alpha[0], m.alpha[1]) + 1) } },
    { kicker: "Objective 1 and 2", title: "Survey, then collect",
      text: "Drive to the BLUE survey beacon on the crater rim, then on to the GREEN sample site. Stop next to the capsule and hold E for three seconds to drill and cache the core.",
      cam: { pos: m2t(sample[0] - 7, sample[1] - 6, g(sample[0], sample[1]) + 5), target: m2t(sample[0], sample[1], g(sample[0], sample[1]) + 0.6) } },
    { kicker: "Objective 3", title: "Bring it home",
      text: "Carry the sample back to the YELLOW beacon at the lander before the battery runs out. Rock hits, rollovers and a flat battery cost you. Your score counts the phases you finish, your time, and how cleanly you drive.",
      cam: { pos: m2t(m.lander[0] + 9, m.lander[1] + 5, g(m.lander[0], m.lander[1]) + 4.5), target: m2t(m.lander[0] + 3, m.lander[1], g(m.lander[0], m.lander[1]) + 1) } },
  ];
}
