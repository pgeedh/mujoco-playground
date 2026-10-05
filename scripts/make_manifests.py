"""Writes manifest.json (list of files, relative to the scene dir) for every
scene under web/assets/scenes so the browser can download a scene's XML and
meshes into MuJoCo's virtual filesystem."""
import json
from pathlib import Path
for d in sorted(Path("web/assets/scenes").iterdir()):
    if not d.is_dir():
        continue
    files = sorted(str(p.relative_to(d)) for p in d.rglob("*") if p.is_file() and p.name not in ("manifest.json",) and not p.name.startswith("LICENSE"))
    (d / "manifest.json").write_text(json.dumps(files, indent=0))
    print(d.name, len(files), "files")
