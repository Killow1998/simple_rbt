import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from simple_rbt_mujoco.fr_mini import build_fr_mini_xml
from simple_rbt_mujoco.ranger_mini import build_ranger_mini_xml
from simple_rbt_mujoco.ridgeback import build_ridgeback_xml
from simple_rbt_mujoco.wheel_geometry import (
    collision_envelope, wheel_contact_geom_ids, body_environment_contacts,
)


def compile_xml(root):
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def scene(builder):
    root = ET.fromstring(builder())
    ET.SubElement(root.find("worldbody"), "geom", name="floor", type="plane", size="10 10 .1")
    return root, *compile_xml(root)


@pytest.mark.parametrize("builder,deck,halfwidth", [
    (build_fr_mini_xml, "upper_deck", .2025),
    (build_ranger_mini_xml, "chassis_deck", .23),
    (build_ridgeback_xml, "deck", .3965),
])
def test_actual_models_include_decks_wheel_descendants_and_exclude_visuals(builder, deck, halfwidth):
    _, model, data = scene(builder)
    envelope = collision_envelope(model, data)
    assert envelope["outer_half_width_m"] == pytest.approx(halfwidth)
    assert deck in envelope["collision_geom_names"]
    wheels = wheel_contact_geom_ids(model)
    assert model.geom("chassis").id not in wheels
    assert model.geom(deck).id not in wheels
    assert set(wheels).issubset(envelope["collision_geom_ids"])
    assert body_environment_contacts(model, data, wheels) == []
    # True underside contact is a failure even though tire contacts are normal.
    data.qpos[2] -= .15
    mujoco.mj_forward(model, data)
    assert "chassis / floor" in body_environment_contacts(model, data)
    assert body_environment_contacts(model, data) == sorted(set(body_environment_contacts(model, data)))


def test_rear_axle_origin_keeps_asymmetric_fr_bounds_and_world_pose_invariance():
    _, model, data = scene(build_fr_mini_xml)
    initial = collision_envelope(model, data)
    assert initial["lower_base_m"][0] == pytest.approx(-.1075)
    assert initial["upper_base_m"][0] == pytest.approx(.5075)
    assert initial["lower_base_m"][0] != -initial["upper_base_m"][0]
    data.qpos[:3] += [2.3, -1.7, .8]
    data.qpos[3:7] = [math.cos(.6), 0, 0, math.sin(.6)]
    mujoco.mj_forward(model, data)
    transformed = collision_envelope(model, data)
    np.testing.assert_allclose(transformed["lower_base_m"], initial["lower_base_m"], atol=1e-12)
    np.testing.assert_allclose(transformed["upper_base_m"], initial["upper_base_m"], atol=1e-12)


@pytest.mark.parametrize("builder,deck", [
    (build_fr_mini_xml, "upper_deck"), (build_ranger_mini_xml, "chassis_deck"),
    (build_ridgeback_xml, "deck"),
])
def test_deck_only_environment_contact_cannot_hide_behind_wheel_exclusion(builder, deck):
    root, model, data = scene(builder)
    geom = model.geom(deck)
    position = data.geom_xpos[geom.id].copy()
    position[1] += float(geom.size[1]) - .003
    ET.SubElement(root.find("worldbody"), "geom", name="deck_probe", type="box",
                  pos=" ".join(str(x) for x in position), size=".003 .003 .003")
    model, data = compile_xml(root)
    contacts = body_environment_contacts(model, data)
    assert f"{deck} / deck_probe" in contacts
    assert "chassis / deck_probe" not in contacts


def test_rollers_count_as_wheel_contacts_and_disabled_hubs_are_excluded():
    root = ET.fromstring(build_ridgeback_xml())
    for geom in root.findall(".//geom"):
        if geom.get("name", "").endswith("_hub"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
    model, data = compile_xml(root)
    wheels = wheel_contact_geom_ids(model)
    assert len(wheels) == 40
    assert all("_roller_" in model.geom(gid).name for gid in wheels)
    bounds = collision_envelope(model, data)
    assert not any(name.endswith("_hub") for name in bounds["collision_geom_names"])
    # A new roller joint pose must be read from data, not inferred from defaults.
    for index in range(model.njnt):
        if (model.joint(index).name or "").endswith("_wheel_joint"):
            data.qpos[model.jnt_qposadr[index]] = .19
    mujoco.mj_forward(model, data)
    turned = collision_envelope(model, data)
    assert turned["lower_base_m"][2] != pytest.approx(bounds["lower_base_m"][2], abs=1e-7)


@pytest.mark.parametrize("kind,size,expected", [
    ("box", ".2 .3 .4", [(.2 + .4) / math.sqrt(2), .3, (.2 + .4) / math.sqrt(2)]),
    ("ellipsoid", ".2 .3 .4", [math.sqrt((.2**2 + .4**2) / 2), .3, math.sqrt((.2**2 + .4**2) / 2)]),
    ("cylinder", ".2 .4", [(.2 + .4) / math.sqrt(2), .2, (.2 + .4) / math.sqrt(2)]),
    ("capsule", ".2 .4", [.2 + .4 / math.sqrt(2), .2, .2 + .4 / math.sqrt(2)]),
    ("sphere", ".2", [.2, .2, .2]),
])
def test_exact_rotated_primitive_support_uses_site_origin_and_ignores_visuals(kind, size, expected):
    root = ET.fromstring(f'''<mujoco><worldbody><body name="base" pos="2 3 4">
      <freejoint/><site name="base_origin" pos=".1 .2 .3"/>
      <geom name="shape" type="{kind}" size="{size}" pos=".4 .5 .6" euler="0 45 0"/>
      <geom name="visual" type="sphere" size="100" contype="0" conaffinity="0" mass="0"/>
    </body><geom name="unrelated_world" type="sphere" size="100"/></worldbody></mujoco>''')
    model, data = compile_xml(root)
    qpos_before, qvel_before = data.qpos.copy(), data.qvel.copy()
    envelope = collision_envelope(model, data)
    np.testing.assert_allclose(envelope["lower_base_m"], .3 - np.array(expected), atol=1e-12)
    np.testing.assert_allclose(envelope["upper_base_m"], .3 + np.array(expected), atol=1e-12)
    assert envelope["collision_geom_names"] == ["shape"]
    np.testing.assert_array_equal(data.qpos, qpos_before)
    np.testing.assert_array_equal(data.qvel, qvel_before)


def test_contact_enabled_means_either_mask_and_other_body_floor_contacts_are_ignored():
    root, _, _ = scene(build_ranger_mini_xml)
    base = root.find(".//body[@name='base']")
    ET.SubElement(base, "geom", name="affinity_only", type="sphere", pos="0 .8 0",
                  size=".01", contype="0", conaffinity="1", mass=".001")
    other = ET.SubElement(root.find("worldbody"), "body", name="other", pos="3 0 .05")
    ET.SubElement(other, "freejoint")
    ET.SubElement(other, "geom", name="other_sphere", type="sphere", size=".1", mass="1")
    model, data = compile_xml(root)
    envelope = collision_envelope(model, data)
    assert "affinity_only" in envelope["collision_geom_names"]
    assert "other_sphere" not in envelope["collision_geom_names"]
    assert envelope["outer_half_width_m"] == pytest.approx(.81)
    assert any(model.geom(c.geom1).name == "other_sphere" or model.geom(c.geom2).name == "other_sphere"
               for c in data.contact)
    assert body_environment_contacts(model, data) == []


@pytest.mark.parametrize("axis,expected_y", [
    ("yaw", (.4 + .2) / math.sqrt(2)),
    ("roll", (.2 + .3) / math.sqrt(2)),
])
def test_world_bounds_include_heading_and_roll_projections_around_offset_site(axis, expected_y):
    root = ET.fromstring('''<mujoco><worldbody><body name="base" pos="2 3 4">
      <freejoint/><site name="base_origin" pos=".1 .2 .3"/>
      <geom name="shape" type="box" pos=".1 .2 .3" size=".4 .2 .3"/>
    </body></worldbody></mujoco>''')
    model, data = compile_xml(root)
    angle = math.pi / 8
    data.qpos[3:7] = ([math.cos(angle), 0, 0, math.sin(angle)] if axis == "yaw" else
                      [math.cos(angle), math.sin(angle), 0, 0])
    mujoco.mj_forward(model, data)
    default = collision_envelope(model, data)
    explicit = collision_envelope(model, data, reference_frame="base_origin")
    world = collision_envelope(model, data, reference_frame="world")
    assert default == explicit
    np.testing.assert_allclose(default["lower_m"], [-.4, -.2, -.3], atol=1e-12)
    np.testing.assert_allclose(default["upper_m"], [.4, .2, .3], atol=1e-12)
    assert world["reference_frame"] == "world"
    assert world["outer_half_width_m"] == pytest.approx(expected_y)
    assert world["lower_m"][1] == pytest.approx(-expected_y)
    assert world["upper_m"][1] == pytest.approx(expected_y)
    assert world["lower_m"] == world["lower_base_m"]
    assert world["upper_m"] == world["upper_base_m"]
    np.testing.assert_allclose(world["origin_world_m"], data.site("base_origin").xpos)
    # Translating the whole robot changes the reported origin, not its bounds.
    data.qpos[:3] += [5, -2, .8]
    mujoco.mj_forward(model, data)
    shifted = collision_envelope(model, data, reference_frame="world")
    np.testing.assert_allclose(shifted["lower_m"], world["lower_m"], atol=1e-12)
    np.testing.assert_allclose(shifted["upper_m"], world["upper_m"], atol=1e-12)
    with pytest.raises(ValueError, match="reference_frame"):
        collision_envelope(model, data, reference_frame="chassis")


def test_ranger_ninety_degree_steering_expands_current_world_y_envelope():
    _, model, data = scene(build_ranger_mini_xml)
    initial = collision_envelope(model, data, reference_frame="world")
    assert initial["outer_half_width_m"] == pytest.approx(.23)
    for name in ("front_left", "front_right", "rear_left", "rear_right"):
        data.qpos[model.joint(name + "_steering_joint").qposadr[0]] = math.pi / 2
    mujoco.mj_forward(model, data)
    steered = collision_envelope(model, data, reference_frame="world")
    # Wheel radius, not half the tire width, now projects along global Y.
    assert steered["lower_m"][1] == pytest.approx(-(.370 / 2 + .110))
    assert steered["upper_m"][1] == pytest.approx(.370 / 2 + .110)
    assert steered["outer_half_width_m"] == pytest.approx(.295)
    data.qpos[3:7] = [math.cos(math.pi / 8), 0, 0, math.sin(math.pi / 8)]
    mujoco.mj_forward(model, data)
    rotated = collision_envelope(model, data, reference_frame="world")
    # At 45 degrees the deck dominates Y: its half-length is now material.
    assert rotated["outer_half_width_m"] == pytest.approx((.350 + .230) / math.sqrt(2))
    assert collision_envelope(model, data)["outer_half_width_m"] == pytest.approx(.295)


def test_fr_rear_axle_world_bounds_keep_the_rotated_body_offset():
    _, model, data = scene(build_fr_mini_xml)
    data.qpos[3:7] = [math.cos(math.pi / 8), 0, 0, math.sin(math.pi / 8)]
    mujoco.mj_forward(model, data)
    world = collision_envelope(model, data, reference_frame="world")
    assert world["lower_m"][1] == pytest.approx((-.1075 - .2025) / math.sqrt(2))
    assert world["upper_m"][1] == pytest.approx((.5075 + .2025) / math.sqrt(2))
    assert world["outer_half_width_m"] == pytest.approx((.5075 + .2025) / math.sqrt(2))


def test_diffdrive_explicit_support_roles_are_normal_but_same_shape_body_is_not():
    from simple_rbt_mujoco.core import _diffdrive_xml
    from simple_rbt_mujoco.wheel_geometry import caster_contact_geom_ids
    root, model, data = scene(_diffdrive_xml)
    data.qpos[2] = .154
    mujoco.mj_forward(model, data)
    supports = caster_contact_geom_ids(model)
    assert {model.geom(gid).name for gid in supports} == {
        "front_caster_collision", "rear_caster_collision"}
    touching = {int(gid) for contact in data.contact if contact.dist <= 1e-6
                for gid in (contact.geom1, contact.geom2)}
    assert set(supports) <= touching
    assert set(supports) <= set(wheel_contact_geom_ids(model))
    assert body_environment_contacts(model, data) == []
    # True chassis floor contact with z > .1 and tilt < 50, so neither of
    # the independent safety limits can conceal a missing contact failure.
    data.qpos[2] = .14
    data.qpos[3:7] = [math.cos(math.pi / 8), math.sin(math.pi / 8), 0, 0]
    mujoco.mj_forward(model, data)
    assert "chassis / floor" in body_environment_contacts(model, data)
    # Add a third base sphere with exactly the same size/friction/height.
    base = root.find(".//body[@name='base']")
    ET.SubElement(base, "geom", name="body_sphere", type="sphere", size=".025",
                  pos="0 .1 -.13", mass=".01", friction=".001 .001 .001")
    model, data = compile_xml(root)
    data.qpos[2] = .154
    mujoco.mj_forward(model, data)
    assert model.geom("body_sphere").id not in wheel_contact_geom_ids(model)
    assert body_environment_contacts(model, data) == ["body_sphere / floor"]


@pytest.mark.parametrize("change", ["missing", "wrong_shape", "wrong_body", "disabled"])
def test_support_role_validation_fails_closed_for_ambiguous_models(change):
    from simple_rbt_mujoco.core import _diffdrive_xml
    from simple_rbt_mujoco.wheel_geometry import caster_contact_geom_ids
    root = ET.fromstring(_diffdrive_xml())
    base = root.find(".//body[@name='base']")
    caster = base.find("geom[@name='front_caster_collision']")
    if change == "missing":
        caster.set("name", "ordinary_body_sphere")
    elif change == "wrong_shape":
        caster.set("type", "box")
        caster.set("size", ".025 .025 .025")
    elif change == "disabled":
        caster.set("contype", "0")
        caster.set("conaffinity", "0")
    else:
        base.remove(caster)
        base.find("body").append(caster)
    model, _ = compile_xml(root)
    with pytest.raises(ValueError, match="[Ss]upport"):
        caster_contact_geom_ids(model)


def test_wheel_model_without_optional_casters_retains_normal_contact_semantics():
    from simple_rbt_mujoco.wheel_geometry import caster_contact_geom_ids
    _, model, data = scene(build_fr_mini_xml)
    assert caster_contact_geom_ids(model) == []
    for _ in range(50):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    assert data.ncon > 0
    assert body_environment_contacts(model, data) == []
