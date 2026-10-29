"""Convert the official Avia assembly, excluding its nonphysical FOV solid."""
import argparse
import importlib.util
import json
import shutil
import subprocess
import sys
import urllib.request
from prepare_visual_assets import ROOT, CACHE, dependencies, export, sha256

def prepare_avia_parts(source, destination):
    import numpy as np
    import trimesh
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDF import TDF_LabelSequence
    from OCP.TDataStd import TDataStd_Name
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopoDS import TopoDS
    from OCP.TopLoc import TopLoc_Location
    from OCP.BRep import BRep_Tool
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.ReadFile(str(source))
    document = TDocStd_Document(TCollection_ExtendedString("Avia"))
    if not reader.Transfer(document):
        raise RuntimeError("Cannot import official Avia CAD")
    tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = TDF_LabelSequence(); tool.GetFreeShapes(roots)
    children = TDF_LabelSequence(); tool.GetComponents_s(roots.Value(1), children)
    shape = None
    for i in range(1, children.Length() + 1):
        label = children.Value(i); name = TDataStd_Name()
        label.FindAttribute(TDataStd_Name.GetID_s(), name)
        if name.Get().ToExtString() == "LIVOX_AVIA":
            shape = tool.GetShape_s(label)
    if shape is None:
        raise RuntimeError("Official Avia device component missing")
    BRepMesh_IncrementalMesh(shape, .15, False, .25, True)
    vertices, faces = [], []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current()); location = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, location); offset = len(vertices)
        if tri is None:
            raise RuntimeError("Avia CAD face was not tessellated")
        for i in range(1, tri.NbNodes() + 1):
            p = tri.Node(i).Transformed(location.Transformation())
            vertices.append([p.X(), p.Y(), p.Z()])
        for i in range(1, tri.NbTriangles() + 1):
            triangle = list(tri.Triangle(i).Get())
            if face.Orientation() == TopAbs_REVERSED: triangle.reverse()
            faces.append([offset + v - 1 for v in triangle])
        explorer.Next()
    vertices = np.asarray(vertices)
    # FOV spherical surfaces share this optical origin. Manual p.16 confirms
    # the 32.4 mm mounting-plane height and +X forward, +Z upward convention.
    origin = np.array([45.49, 48.30000000001, -38.502379674186024])
    rotation = np.array([[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
    local = (vertices - origin) @ rotation.T * .001
    mesh = trimesh.Trimesh(vertices=local, faces=faces, process=True)
    mesh.update_faces(mesh.unique_faces()); mesh.remove_unreferenced_vertices()
    window = (mesh.triangles_center[:,0] > .02399) & (mesh.triangles_center[:,0] < .02429)
    parts = []
    for name, mask, color in [("avia-glass", window, [18,24,31,255]),
                              ("avia-housing", ~window, [155,158,161,255])]:
        path = source.parent/(name + '.stl')
        mesh.submesh([np.flatnonzero(mask)], append=True).export(path)
        parts.append(export({'part':name, 'color':color, 'target_faces':12000}, path, destination))
    return parts


def prepare(refresh=False):
    destination = ROOT / "assets/visual/sensors"
    manifest = destination / "avia-provenance.json"
    data = json.loads(manifest.read_text())
    lidar = ROOT / "assets/lidar"
    sources = json.loads((lidar / "avia-source.json").read_text())["sources"]
    for name, filename in [("avia.csv", "avia.csv"), ("livox_sim_LICENSE", "avia.LICENSE")]:
        path, source = lidar / filename, sources[name]
        if not path.is_file() or sha256(path) != source["sha256"]:
            payload = urllib.request.urlopen(source["url"], timeout=90).read()
            import hashlib
            if hashlib.sha256(payload).hexdigest() != source["sha256"]:
                raise RuntimeError("Avia source checksum mismatch: " + name)
            path.write_bytes(payload)
    if not refresh and all((ROOT / f["file"]).is_file() and sha256(ROOT / f["file"]) == f["sha256"]
                           for part in data["parts"] for f in part["files"]):
        print("Avia: verified official scan pattern and derived meshes")
        return
    source = data["source_files"][0]
    cache = ROOT / "build/sensor-preparation"
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / source["name"]
    if not path.is_file() or sha256(path) != source["sha256"]:
        path.write_bytes(urllib.request.urlopen(source["url"], timeout=90).read())
    if sha256(path) != source["sha256"]:
        raise RuntimeError("Avia CAD checksum mismatch")
    dependencies()
    if importlib.util.find_spec("OCP") is None:
        subprocess.run([shutil.which("uv") or "uv", "pip", "install", "--python", sys.executable,
                        "--target", str(CACHE / "python"), "cadquery-ocp-novtk==7.9.3.1.1"], check=True)
    parts = prepare_avia_parts(path, destination)
    if not refresh:
        expected = {f["file"]: f["sha256"] for p in data["parts"] for f in p["files"]}
        actual = {f["file"]: f["sha256"] for p in parts for f in p["files"]}
        if actual != expected:
            raise RuntimeError("Avia derived meshes differ from pinned assets")
    data["parts"] = parts
    manifest.write_text(json.dumps(data, indent=2) + "\n")
    print("Avia: prepared official meshes")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--refresh-manifests", action="store_true")
    prepare(parser.parse_args().refresh_manifests)
