// Optional "World Labs" worlds: if a scene folder is dropped into
// web/assets/worlds/<env id>/ it is shown as the visual backdrop for that
// environment (physics still comes from the MuJoCo scene). Layout:
//   web/assets/worlds/<env id>/world.json   { "file": "scene.glb", "position": [x,y,z], "rotation": [rx,ry,rz], "scale": 1, "hide": ["body name", ...] }
//   web/assets/worlds/<env id>/scene.glb    a mesh exported from World Labs (Marble) or any glTF
// Coordinates are three.js (y up). `hide` lists MuJoCo body names whose default
// visuals should be hidden (e.g. the stand-in terrain) so the world shows through.
import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

export async function loadWorld(demo, envId) {
  if (demo.worldRoot) { demo.scene.remove(demo.worldRoot); demo.worldRoot = null; }
  let cfg;
  try {
    const r = await fetch(`./assets/worlds/${envId}/world.json`);
    if (!r.ok) return false;
    cfg = await r.json();
  } catch { return false; }
  try {
    const gltf = await new GLTFLoader().loadAsync(`./assets/worlds/${envId}/${cfg.file}`);
    const root = new THREE.Group();
    root.name = "WorldLabsWorld";
    root.add(gltf.scene);
    const [px, py, pz] = cfg.position || [0, 0, 0], [rx, ry, rz] = cfg.rotation || [0, 0, 0];
    root.position.set(px, py, pz); root.rotation.set(rx, ry, rz); root.scale.setScalar(cfg.scale || 1);
    demo.scene.add(root); demo.worldRoot = root;
    for (const name of cfg.hide || []) {
      for (const b of Object.values(demo.bodies)) if (b && b.name === name) b.visible = false;
    }
    return true;
  } catch (e) { console.warn("World Labs world failed to load:", e); return false; }
}
