#!/usr/bin/env python3
"""Prepare checksum-pinned Livox/RealSense visuals; no runtime CAD dependency."""
import argparse
import json
import sys
import urllib.request
from pathlib import Path
from prepare_visual_assets import ROOT, CACHE, dependencies, export, sha256

DEST = ROOT / "assets/visual/sensors"
SOURCE = ROOT / "build/sensor-preparation"

def realsense_parts():
    import math, numpy as np, trimesh
    scene=trimesh.load(SOURCE/"d435.dae",process=False,force="scene")
    groups={"housing":[],"window":[],"optics":[]}
    for node in scene.graph.nodes_geometry:
        placement,name=scene.graph.get(node)
        mesh=scene.geometry[name].copy();mesh.apply_transform(placement)
        if name=="ID77":
            # The official CAD has an untextured front cover. Keep its exact
            # geometry, using translucent glass so recessed optics remain visible.
            front=mesh.triangles_center[:,2]>-.004
            for label,mask in [("window",front),("housing",~front)]:
                groups[label].append(mesh.submesh([np.flatnonzero(mask)],append=True))
        else:
            groups["optics" if name in {"ID29","ID71","ID89","ID95","ID101"} else "housing"].append(mesh)
    result=[]
    for label,color in [("housing",[170,175,180,255]),("window",[18,24,31,235]),("optics",[20,26,33,255])]:
        source=SOURCE/("d435-"+label+".stl")
        trimesh.util.concatenate(groups[label]).export(source)
        result.append(export({"part":"d435-"+label,"color":color,"rpy":[math.pi/2,0,math.pi/2],
                              "position":[.0043,-.0175,0],"target_faces":12000 if label=="housing" else 3000},source,DEST))
    return result

def prepare(refresh=False):
    manifest = DEST / "provenance.json"
    data = json.loads(manifest.read_text())
    if not refresh and data["parts"] and all((ROOT/f["file"]).is_file() and sha256(ROOT/f["file"]) == f["sha256"] for part in data["parts"] for f in part["files"]):
        print("sensors: verified official derived meshes"); return
    SOURCE.mkdir(parents=True, exist_ok=True)
    for source in data["source_files"]:
        path = SOURCE/source["name"]
        if not path.exists() or sha256(path) != source["sha256"]:
            path.write_bytes(urllib.request.urlopen(source["url"], timeout=60).read())
        if sha256(path) != source["sha256"]: raise RuntimeError("Sensor source checksum mismatch: "+source["name"])
    dependencies()
    import importlib.metadata, subprocess, shutil
    version="7.9.3.1.1"
    try: installed=importlib.metadata.version("cadquery-ocp-novtk")
    except importlib.metadata.PackageNotFoundError: installed=""
    if installed!=version:
        subprocess.run([shutil.which("uv") or "uv","pip","install","--python",sys.executable,
                        "--target",str(CACHE/"python"),"cadquery-ocp-novtk=="+version],check=True)
    import numpy as np, trimesh
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopoDS import TopoDS
    from OCP.TopLoc import TopLoc_Location
    from OCP.BRep import BRep_Tool
    reader=STEPControl_Reader()
    if reader.ReadFile(str(SOURCE/"mid360.stp"))!=IFSelect_RetDone: raise RuntimeError("Cannot read official STEP")
    reader.TransferRoots();shape=reader.OneShape()
    # Traverse faces, including open shells. Importing only STEP solids drops
    # the MID360 side housing even though the file was read without an error.
    BRepMesh_IncrementalMesh(shape,.25,False,.3,True)
    vertices=[];faces=[];explorer=TopExp_Explorer(shape,TopAbs_FACE)
    while explorer.More():
        face=TopoDS.Face_s(explorer.Current());location=TopLoc_Location()
        triangulation=BRep_Tool.Triangulation_s(face,location)
        if triangulation is None: raise RuntimeError("Official CAD face was not tessellated")
        offset=len(vertices)
        for i in range(1,triangulation.NbNodes()+1):
            point=triangulation.Node(i).Transformed(location.Transformation())
            vertices.append([point.X(),point.Y(),point.Z()])
        for i in range(1,triangulation.NbTriangles()+1):
            triangle=list(triangulation.Triangle(i).Get())
            if face.Orientation()==TopAbs_REVERSED: triangle.reverse()
            faces.append([offset+v-1 for v in triangle])
        explorer.Next()
    vertices=np.asarray(vertices);bottom=float(vertices[:,1].min())
    # Manual p.9/12: optical origin 47 mm above mounting plane, +X away
    # from the connector. STEP uses millimetres with +Y upright.
    vertices=np.column_stack((-vertices[:,0],vertices[:,2],vertices[:,1]-bottom-47.))*.001
    mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
    mesh.update_faces(mesh.unique_faces());mesh.remove_unreferenced_vertices()
    parts=[];lens=mesh.triangles_center[:,2]>-.0075
    for label,mask,color in [("mid360-glass",lens,[28,32,38,255]),("mid360-housing",~lens,[170,174,178,255])]:
        subset=mesh.submesh([np.flatnonzero(mask)],append=True)
        path=SOURCE/(label+".stl");subset.export(path)
        parts.append(export({"part":label,"color":color,"target_faces":9000},path,DEST))
    alignment={"cad_bottom_y_mm":bottom,"lidar_origin_height_mm":47.,"rotation":[[-1,0,0],[0,0,1],[0,1,0]],"bounds_m":mesh.bounds.tolist()}
    # Exact visual transform in official _d435.urdf.xacro, relative to camera_link.
    parts.extend(realsense_parts())
    if not refresh:
        old={f["file"]:f["sha256"] for p in data["parts"] for f in p["files"]}
        new={f["file"]:f["sha256"] for p in parts for f in p["files"]}
        if old != new: raise RuntimeError("Derived sensor mesh checksum mismatch")
    else:
        data["parts"]=parts;data["mid360_alignment"]=alignment
        manifest.write_text(json.dumps(data,indent=2)+"\n")
    print("sensors: prepared official meshes",sum(f["bytes"] for p in parts for f in p["files"]),"bytes")

if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--refresh-manifests",action="store_true")
    prepare(parser.parse_args().refresh_manifests)
