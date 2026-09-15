// Our own scene manifest + downloader, instead of the demo's hardcoded
// example-scenes list in mujocoUtils.js. Kept separate so we don't have to
// keep patching vendored code as we add more scenes/assets later.

export const G1_PARK_SCENE = "g1_park/world.xml";

const G1_PARK_FILES = [
  "g1_park/assets/head_link.STL",
  "g1_park/assets/left_ankle_pitch_link.STL",
  "g1_park/assets/left_ankle_roll_link.STL",
  "g1_park/assets/left_elbow_link.STL",
  "g1_park/assets/left_hand_index_0_link.STL",
  "g1_park/assets/left_hand_index_1_link.STL",
  "g1_park/assets/left_hand_middle_0_link.STL",
  "g1_park/assets/left_hand_middle_1_link.STL",
  "g1_park/assets/left_hand_palm_link.STL",
  "g1_park/assets/left_hand_thumb_0_link.STL",
  "g1_park/assets/left_hand_thumb_1_link.STL",
  "g1_park/assets/left_hand_thumb_2_link.STL",
  "g1_park/assets/left_hip_pitch_link.STL",
  "g1_park/assets/left_hip_roll_link.STL",
  "g1_park/assets/left_hip_yaw_link.STL",
  "g1_park/assets/left_knee_link.STL",
  "g1_park/assets/left_rubber_hand.STL",
  "g1_park/assets/left_shoulder_pitch_link.STL",
  "g1_park/assets/left_shoulder_roll_link.STL",
  "g1_park/assets/left_shoulder_yaw_link.STL",
  "g1_park/assets/left_wrist_pitch_link.STL",
  "g1_park/assets/left_wrist_roll_link.STL",
  "g1_park/assets/left_wrist_yaw_link.STL",
  "g1_park/assets/logo_link.STL",
  "g1_park/assets/pelvis.STL",
  "g1_park/assets/pelvis_contour_link.STL",
  "g1_park/assets/right_ankle_pitch_link.STL",
  "g1_park/assets/right_ankle_roll_link.STL",
  "g1_park/assets/right_elbow_link.STL",
  "g1_park/assets/right_hand_index_0_link.STL",
  "g1_park/assets/right_hand_index_1_link.STL",
  "g1_park/assets/right_hand_middle_0_link.STL",
  "g1_park/assets/right_hand_middle_1_link.STL",
  "g1_park/assets/right_hand_palm_link.STL",
  "g1_park/assets/right_hand_thumb_0_link.STL",
  "g1_park/assets/right_hand_thumb_1_link.STL",
  "g1_park/assets/right_hand_thumb_2_link.STL",
  "g1_park/assets/right_hip_pitch_link.STL",
  "g1_park/assets/right_hip_roll_link.STL",
  "g1_park/assets/right_hip_yaw_link.STL",
  "g1_park/assets/right_knee_link.STL",
  "g1_park/assets/right_rubber_hand.STL",
  "g1_park/assets/right_shoulder_pitch_link.STL",
  "g1_park/assets/right_shoulder_roll_link.STL",
  "g1_park/assets/right_shoulder_yaw_link.STL",
  "g1_park/assets/right_wrist_pitch_link.STL",
  "g1_park/assets/right_wrist_roll_link.STL",
  "g1_park/assets/right_wrist_yaw_link.STL",
  "g1_park/assets/torso_link_rev_1_0.STL",
  "g1_park/assets/waist_roll_link_rev_1_0.STL",
  "g1_park/assets/waist_yaw_link_rev_1_0.STL",
  "g1_park/g1.xml",
  "g1_park/g1_with_hands.xml",
  "g1_park/world.xml",
];

function isBinaryAsset(path) {
  const lower = path.toLowerCase();
  return lower.endsWith(".png") || lower.endsWith(".stl") || lower.endsWith(".skn");
}

export async function downloadG1ParkScene(mujoco) {
  const responses = await Promise.all(
    G1_PARK_FILES.map((path) => fetch("./assets/scenes/" + path))
  );

  for (let i = 0; i < responses.length; i++) {
    if (!responses[i].ok) {
      throw new Error(`Failed to fetch ${G1_PARK_FILES[i]}: ${responses[i].status}`);
    }
    const parts = G1_PARK_FILES[i].split("/");
    let working = "/working/";
    for (let f = 0; f < parts.length - 1; f++) {
      working += parts[f];
      if (!mujoco.FS.analyzePath(working).exists) { mujoco.FS.mkdir(working); }
      working += "/";
    }

    if (isBinaryAsset(G1_PARK_FILES[i])) {
      mujoco.FS.writeFile("/working/" + G1_PARK_FILES[i], new Uint8Array(await responses[i].arrayBuffer()));
    } else {
      mujoco.FS.writeFile("/working/" + G1_PARK_FILES[i], await responses[i].text());
    }
  }
}
