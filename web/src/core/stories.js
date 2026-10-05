// Storyboard registry: maps an environment's `story` key to its steps.
import { g1Story } from "../robots/g1/story.js";
import { frankaStory } from "../robots/franka/story.js";
import { huskyStory } from "../robots/husky/story.js";

const CONTROLS = (env) => ({ kicker: "Controls", title: "Ready?", text: env.help.replace(/&nbsp;/g, " "), orbit: 0 });

export function storyFor(env, controller) {
  const steps = env.story === "g1" ? g1Story(controller) : env.story === "franka" ? frankaStory(controller) : huskyStory(controller);
  const last = steps[steps.length - 1];
  return [...steps, { ...CONTROLS(env), cam: { pos: last.cam.pos, target: last.cam.target } }];
}
