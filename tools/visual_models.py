"""Offline visual-only overlays; dynamics and sensor mounts remain original."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def apply_visual(root, robot, project_root):
    provenance = Path(project_root) / "assets/visual" / robot / "provenance.json"
    if not provenance.is_file():
        return
    data = json.loads(provenance.read_text())
    overlay = data.get("mjcf_overlay")
    if not overlay:
        return
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    bodies = {body.get("name"): body for body in root.iter("body")}
    geoms = {geom.get("name"): geom for geom in root.iter("geom")}
    for name in overlay["hide_geoms"]:
        geom = geoms[name]
        color = geom.get("rgba", "0.5 0.5 0.5 1").split()
        color[3] = "0"
        geom.set("rgba", " ".join(color))
    parts = {part["part"]: part["files"] for part in data["parts"]}
    meshes = {}
    for index, attachment in enumerate(overlay["attachments"]):
        for number, source in enumerate(parts[attachment["part"]]):
            name = f"official_{robot}_{index}_{number}"
            scale = tuple(attachment.get("scale", [1, 1, 1]))
            identity = (source["file"], scale)
            mesh = meshes.get(identity)
            if mesh is None:
                mesh = name + "_mesh"
                ET.SubElement(asset, "mesh", name=mesh,
                              file="@PROJECT_ROOT@/" + source["file"], inertia="shell",
                              scale=" ".join(map(str, scale)))
                meshes[identity] = mesh
            attributes = dict(name=name, type="mesh", mesh=mesh, mass="0",
                              contype="0", conaffinity="0", group="2",
                              rgba=" ".join(map(str, attachment.get("rgba", source["rgba"]))),
                              pos=" ".join(map(str, attachment.get("position", [0, 0, 0]))),
                              quat=" ".join(map(str, attachment.get("quaternion", [1, 0, 0, 0]))))
            ET.SubElement(bodies[attachment["body"]], "geom", **attributes)


def update_models(project_root):
    import hashlib
    project_root = Path(project_root)
    catalog_path = project_root / "assets/catalog.json"
    catalog = json.loads(catalog_path.read_text())
    for key, entry in catalog["models"].items():
        robot = key.split("-", 1)[0]
        provenance = project_root / "assets/visual" / robot / "provenance.json"
        if not provenance.is_file() or not json.loads(provenance.read_text()).get("mjcf_overlay"):
            continue
        path = project_root / "assets" / entry["file"]
        model = ET.parse(path)
        # This command overlays pristine exported XML once; reject duplicates.
        if any((geom.get("name") or "").startswith("official_") for geom in model.iter("geom")):
            continue
        apply_visual(model.getroot(), robot, project_root)
        ET.indent(model, space="  ")
        model.write(path, encoding="unicode")
        with path.open("a") as output:
            output.write("\n")
        entry["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    catalog_path.write_text(json.dumps(catalog, indent=2) + "\n")


if __name__ == "__main__":
    update_models(Path(__file__).resolve().parents[1])
