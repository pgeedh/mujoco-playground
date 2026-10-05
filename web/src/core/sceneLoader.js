// Downloads one scene (its XML + meshes, listed in manifest.json) into
// MuJoCo's virtual filesystem under /working/<dir>/. Scenes are fetched the
// first time they're opened and cached for the rest of the session.
const loaded = new Set();

export async function downloadScene(mujoco, dir) {
  if (loaded.has(dir)) return;
  const base = `./assets/scenes/${dir}/`;
  const manifest = await (await fetch(base + "manifest.json")).json();
  // Fetch with a small worker pool (browsers cap parallel connections per host anyway).
  const responses = new Array(manifest.length);
  let next = 0;
  const worker = async () => {
    while (next < manifest.length) {
      const i = next++;
      const r = await fetch(base + manifest[i]);
      if (!r.ok) throw new Error(`Failed to fetch ${dir}/${manifest[i]}: ${r.status}`);
      responses[i] = new Uint8Array(await r.arrayBuffer());
    }
  };
  await Promise.all([...Array(6)].map(worker));
  for (let i = 0; i < manifest.length; i++) {
    const parts = (dir + "/" + manifest[i]).split("/");
    let working = "/working";
    for (let p = 0; p < parts.length - 1; p++) {
      working += "/" + parts[p];
      if (!mujoco.FS.analyzePath(working).exists) mujoco.FS.mkdir(working);
    }
    mujoco.FS.writeFile("/working/" + dir + "/" + manifest[i], responses[i]);
  }
  loaded.add(dir);
}
