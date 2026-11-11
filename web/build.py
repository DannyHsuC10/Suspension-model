"""Assemble the static site and package the shared Python modules for Pyodide."""
import argparse
from hashlib import sha256
from pathlib import Path
from shutil import copyfile
from zipfile import ZIP_DEFLATED, ZipFile


def build(destination):
    web = Path(__file__).resolve().parent
    package = web.parent / "suspension_dynamics"
    destination.mkdir(parents=True, exist_ok=True)
    for name in ("style.css", "app.js", "plots.js", "worker.js", "notes.js", "model-notes.md"):
        copyfile(web / name, destination / name)
    with ZipFile(destination / "engine.zip", "w", ZIP_DEFLATED) as archive:
        for name in ("__init__.py", "catalog.py", "default_project.json", "engine.py",
                     "project.py", "project_import.py", "road_profiles.py", "storage.py"):
            archive.write(package / name, f"suspension_dynamics/{name}")
        archive.write(web / "bridge.py", "web_bridge.py")
        archive.write(web / "english.py", "web/english.py")

    # Keep the page, worker and Python engine on the same release in browser caches.
    revision = sha256()
    for name in ("app.js", "worker.js", "engine.zip"):
        revision.update((destination / name).read_bytes())
    version = revision.hexdigest()[:16]
    index = (web / "index.html").read_text(encoding="utf-8")
    (destination / "index.html").write_text(
        index.replace('src="app.js"', f'src="app.js?v={version}"'), encoding="utf-8",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    build(parser.parse_args().destination)
