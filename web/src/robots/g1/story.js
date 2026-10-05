// Storyboard for the nuclear plant mission.
import { m2t } from "../../core/briefing.js";

export function g1Story(ctl) {
  const c = ctl.mission.cfg, t = c.terminal;
  return [
    { kicker: "Halcyon Nuclear Plant · Unit 3", title: "Coolant leak", stamp: "ALARM LEVEL 7", alarm: true,
      text: "Primary coolant is venting somewhere in the reactor hall. The crew has been evacuated and the doors are sealed. The only thing that can go in is a robot: a Unitree G1 with a dosimeter on its chest.",
      cam: { pos: m2t(-1.0, -9.5, 10.5), target: m2t(c.reactor[0], c.reactor[1], 2.5) } },
    { kicker: "The hall", title: "Steam is not radiation",
      text: "Six pipe couplings are steaming. Only some of them are radioactive and every one looks the same. The dose rate on your gauge climbs as you get closer. Concrete shields and the reactor itself block part of the radiation.",
      cam: { pos: m2t(6, -9.5, 7.5), target: m2t(c.reactor[0] - 1, c.reactor[1] + 1, 1.5) } },
    { kicker: "Mind the dose", title: "Ten grays is all you get",
      text: "Radiation is slowly killing the robot's electronics. Every Gy you absorb counts against a 10 Gy budget, so triangulate from a few metres away instead of standing on top of a leak. Flags only need to land within 3.5 metres.",
      cam: { pos: m2t(11, 6.8, 3.2), target: m2t(8.5, 3.5, 1.2) } },
    { kicker: "Flag and report", title: "Find three leaks, then report",
      text: "Press F next to a leak to plant a flag (three flags in total, G undoes one). When you are done, walk back to the green pad at the control terminal and press Enter to file your report.",
      cam: { pos: m2t(t[0] + 4.5, t[1] - 4, 3), target: m2t(t[0], t[1], 0.8) } },
  ];
}
