// Static build for hosting (Vercel etc.): the app loads its dependencies
// straight from ./node_modules at runtime (no bundler), and node_modules
// isn't deployed, so copy just the runtime files into dist/ with the same
// relative layout.
import { cpSync, rmSync, mkdirSync } from "node:fs";

const OUT = "dist";
rmSync(OUT, { recursive: true, force: true });
mkdirSync(OUT, { recursive: true });

const noMaps = (src) => !src.endsWith(".map");
const copy = (from, to = from, filter = noMaps) =>
  cpSync(from, `${OUT}/${to}`, { recursive: true, filter });

copy("index.html");
copy("src");
copy("assets");
copy("node_modules/three/build", undefined, (s) => noMaps(s) && !s.endsWith(".cjs"));
// three/examples/jsm: everything except the big optional decoder libraries (keep lil-gui)
copy("node_modules/three/examples/jsm", undefined, (s) => noMaps(s) && (!s.includes("/jsm/libs/") || s.includes("lil-gui")));
copy("node_modules/@mujoco/mujoco", undefined, (s) => noMaps(s) && !s.includes("/mujoco/mt"));   // skip the multithreaded build
copy("node_modules/onnxruntime-web/dist/ort.min.js");
copy("node_modules/onnxruntime-web/dist/ort-wasm-simd-threaded.jsep.mjs");
copy("node_modules/onnxruntime-web/dist/ort-wasm-simd-threaded.jsep.wasm");

console.log(`built ${OUT}/`);
