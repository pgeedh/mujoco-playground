// Storyboard for the biohazard lab mission.
import { m2t } from "../../core/briefing.js";

export function frankaStory() {
  return [
    { kicker: "BSL-4 biosafety lab · 02:14", title: "Containment breach", stamp: "LEVEL 4", alarm: true,
      text: "A sample of the engineered NX-7 virus has leaked in the cold room. The staff are out. The building purge starts in four minutes, and everything still on the bench goes to the incinerator, including the only samples that could lead to a vaccine.",
      cam: { pos: m2t(-1.9, -1.9, 1.5), target: m2t(0, 0.1, 0.55) } },
    { kicker: "Two arms, two zones", title: "Neither can do it alone",
      text: "Two Franka arms are all that is left. The LEFT arm can only reach the sample rack on the left. The RIGHT arm can only reach the biohazard case on the right. A dropped tube that leaves the bench is a spill.",
      cam: { pos: m2t(0.1, -1.3, 1.0), target: m2t(0, 0.1, 0.5) } },
    { kicker: "The pass-through", title: "Meet at the airlock",
      text: "They meet in the middle: the left arm sets a tube down in the AIRLOCK tray, then the right arm picks it up. Grip a tube across its width near the middle and carry it slowly; the gripper holds, but not forever.",
      cam: { pos: m2t(0.5, -0.45, 0.8), target: m2t(0, 0.14, 0.43) } },
    { kicker: "Seal it", title: "Four tubes, one button",
      text: "Drop all four tubes into the biohazard case, then press the red SEAL button with a closed gripper. Spills cost you points, a sealed case before the purge scores the most.",
      cam: { pos: m2t(1.3, -0.55, 0.85), target: m2t(0.6, 0.1, 0.43) } },
  ];
}
