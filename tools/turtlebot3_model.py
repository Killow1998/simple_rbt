"""Offline conversion of the pinned Waffle Pi URDF, not a runtime importer."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROBOT = "turtlebot3_waffle_pi"
CONTROLLER = "diff_drive"


def numbers(values):
    return " ".join(format(float(value), ".17g") for value in values)


def vector(text="0 0 0"):
    return [float(value) for value in text.split()]


def transform(element):
    origin = element.find("origin")
    if origin is None:
        return {}
    result = {"pos": origin.get("xyz", "0 0 0")}
    roll, pitch, yaw = vector(origin.get("rpy", "0 0 0"))
    cr, cp, cy = [math.cos(value / 2) for value in (roll, pitch, yaw)]
    sr, sp, sy = [math.sin(value / 2) for value in (roll, pitch, yaw)]
    result["quat"] = numbers([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                              cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])
    return result


def controller_config(project_root):
    manifest = json.loads((Path(project_root) / f"assets/manifests/{ROBOT}_manifest.json").read_text())
    geometry = manifest["geometry"]
    return {"implemented_geometry": {"wheel_radius_m": geometry["wheel_radius_m"],
                                     "track_m": geometry["controller_track_m"],
                                     "physical_track_m": geometry["urdf_track_m"]},
            "wheel_actuators": ["wheel_left_velocity", "wheel_right_velocity"],
            "upstream": manifest["upstream"], "adaptation": manifest["adaptation"]}


def build_model(project_root):
    source = Path(project_root) / f"assets/visual/{ROBOT}"
    urdf = ET.fromstring((source / "turtlebot3_waffle_pi.urdf").read_text().replace("${namespace}", ""))
    links = {link.get("name"): link for link in urdf.findall("link")}
    children = {}
    for joint in urdf.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(joint)
    colors = {m.get("name"): m.find("color").get("rgba") for m in urdf.findall("material")}
    root = ET.Element("mujoco", model=ROBOT)
    root.append(ET.Comment("Offline MJCF adaptation of ROBOTIS turtlebot3 90a68bd2e3c61c12966779da89d8eeaec82730e9; original inertias and collisions retained. Apache-2.0: assets/visual/turtlebot3_waffle_pi/LICENSE."))
    ET.SubElement(root, "compiler", angle="radian", inertiafromgeom="false")
    ET.SubElement(root, "option", timestep="0.001", integrator="implicitfast", cone="elliptic", iterations="50")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "geom", friction="1 .005 .0001", condim="4")
    ET.SubElement(defaults, "joint", damping="0.0001", armature="0.00001")
    asset = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    # *_wheel assemblies identify all support contacts to the existing course
    # gates. The original URDF's two fixed caster collision boxes are retained.
    names = {"base_link": "base", "wheel_left_link": "left_wheel", "wheel_right_link": "right_wheel",
             "caster_back_left_link": "caster_back_left_wheel", "caster_back_right_link": "caster_back_right_wheel"}

    def add_link(name, parent, incoming=None):
        attributes = transform(incoming) if incoming is not None else {"pos": "0 0 .0101"}
        body = ET.SubElement(parent, "body", name=names.get(name, name), **attributes)
        if incoming is None:
            ET.SubElement(body, "freejoint", name="base_free")
        elif incoming.get("type") == "continuous":
            ET.SubElement(body, "joint", name=incoming.get("name"), type="hinge", axis=incoming.find("axis").get("xyz"))
        elif incoming.get("type") != "fixed":
            raise ValueError("Unexpected joint in pinned TurtleBot3 URDF")
        link = links[name]
        inertia = link.find("inertial")
        if inertia is not None:
            origin = inertia.find("origin")
            if origin is not None and vector(origin.get("rpy", "0 0 0")) != [0., 0., 0.]:
                raise ValueError("Pinned TurtleBot3 inertial frame changed")
            tensor = inertia.find("inertia").attrib
            ET.SubElement(body, "inertial", pos=origin.get("xyz", "0 0 0") if origin is not None else "0 0 0",
                          mass=inertia.find("mass").get("value"),
                          fullinertia=" ".join(tensor[key] for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")))
        for kind in ("visual", "collision"):
            for index, shape in enumerate(link.findall(kind)):
                geometry = shape.find("geometry")
                attributes = transform(shape)
                attributes["name"] = name + "_" + kind + (str(index) if index else "")
                if kind == "visual":
                    attributes.update(contype="0", conaffinity="0", group="2")
                    material = shape.find("material")
                    attributes["rgba"] = colors.get(material.get("name") if material is not None else "", ".4 .4 .4 1")
                else:
                    attributes.update(group="3", rgba=".3 .3 .3 0")
                    if name.startswith("caster_"):
                        attributes.update(friction=".001 .0001 .0001", priority="1")
                mesh, box, cylinder = geometry.find("mesh"), geometry.find("box"), geometry.find("cylinder")
                if mesh is not None:
                    mesh_name = name + "_mesh"
                    ET.SubElement(asset, "mesh", name=mesh_name,
                                  file=f"@PROJECT_ROOT@/assets/visual/{ROBOT}/" + Path(mesh.get("filename")).name,
                                  scale=mesh.get("scale", "1 1 1"))
                    attributes.update(type="mesh", mesh=mesh_name)
                elif box is not None:
                    attributes.update(type="box", size=numbers([v/2 for v in vector(box.get("size"))]))
                elif cylinder is not None:
                    attributes.update(type="cylinder", size=numbers([cylinder.get("radius"), float(cylinder.get("length"))/2]))
                else:
                    raise ValueError("Unexpected geometry in pinned TurtleBot3 URDF")
                ET.SubElement(body, "geom", **attributes)
        if name == "base_link":
            for site, position in [("base_origin", "0 0 0"), ("imu", "0 0 .068"),
                                   ("lidar", "-.064 0 .18"), ("camera_mount", ".076 0 .093"),
                                   ("mid360_imu", "-.053 .02329 .13588")]:
                ET.SubElement(body, "site", name=site, pos=position, size=".002", quat="1 0 0 0")
        for joint in children.get(name, []):
            add_link(joint.find("child").get("link"), body, joint)
    add_link("base_link", world)
    actuators = ET.SubElement(root, "actuator")
    for side in ("left", "right"):
        ET.SubElement(actuators, "velocity", name=f"wheel_{side}_velocity", joint=f"wheel_{side}_joint",
                      kv="1", forcelimited="true", forcerange="-1 1")
    sensors = ET.SubElement(root, "sensor")
    for kind, name, site in [("gyro", "gyro", "imu"), ("accelerometer", "accelerometer", "imu"),
                             ("velocimeter", "base_linvel", "base_origin"), ("gyro", "base_angvel", "base_origin"),
                             ("gyro", "mid360_gyro", "mid360_imu"), ("accelerometer", "mid360_accelerometer", "mid360_imu")]:
        ET.SubElement(sensors, kind, name=name, site=site)
    ET.indent(root, space="  ")
    return root


def export_model(project_root):
    project_root = Path(project_root)
    model = project_root / f"assets/models/{ROBOT}-{CONTROLLER}.xml"
    model.write_text(ET.tostring(build_model(project_root), encoding="unicode") + "\n")
    catalog_path = project_root / "assets/catalog.json"
    catalog = json.loads(catalog_path.read_text())
    catalog["models"][f"{ROBOT}-{CONTROLLER}"] = {"file": "models/"+model.name, "sha256": hashlib.sha256(model.read_bytes()).hexdigest()}
    catalog["controllers"][f"{ROBOT}-{CONTROLLER}"] = controller_config(project_root)
    catalog["models"]["diffdrive-none"]["legacy"] = True
    catalog_path.write_text(json.dumps(catalog, indent=2)+"\n")
