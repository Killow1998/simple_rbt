"""Physical surface, ray and contact checks for benchmark terrain courses."""
import json
import math
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from simple_rbt_mujoco.terrains import TERRAIN_LEVELS, TERRAIN_NAMES, append_terrain


def scene(name, **kwargs):
    root = ET.fromstring('''<mujoco><option timestep="0.002"/>
      <worldbody><geom name="floor" type="plane" size="12 6 .1"/></worldbody>
    </mujoco>''')
    profile = append_terrain(root, name, **kwargs)
    return root, profile


def compile_scene(root):
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def physical_height(model, data, x, y):
    geom_id = np.empty(1, dtype=np.int32)
    distance = mujoco.mj_ray(model, data, np.array([x, y, 5.]),
                             np.array([0., 0., -1.]), np.ones(6, dtype=np.uint8),
                             True, -1, geom_id)
    assert distance >= 0., "The floor must intercept every vertical test ray"
    return 5. - distance


def test_default_course_dimensions_and_known_heights():
    _, flat = scene("flat")
    assert flat.height_at(4., 0.) == 0.
    _, ramp = scene("ramp")
    angle = math.radians(8.)
    assert ramp.height_at(2., 0.) == pytest.approx(math.tan(angle))
    assert ramp.height_at(3.3, 0.) == pytest.approx(2. * math.tan(angle))
    assert ramp.height_at(4.6, 0.) == pytest.approx(math.tan(angle))
    assert ramp.height_at(.99, 0.) == 0.
    assert ramp.height_at(5.61, 0.) == 0.
    _, stairs = scene("stairs")
    np.testing.assert_allclose(stairs.height_at([1.2, 1.6, 2., 2.4, 3., 3.6, 4., 4.4, 4.8]),
                               [.06, .12, .18, .24, .24, .18, .12, .06, 0.])
    for name in TERRAIN_NAMES:
        _, profile = scene(name)
        assert profile.finish_x == 6.
        assert profile.exit_x < profile.finish_x
        assert profile.height_at(profile.finish_x) == 0.
        assert profile.height_at(2., 1.21) == 0.
        assert profile.metadata["geom_count"] <= 60
        json.dumps(profile.metadata, allow_nan=False)


@pytest.mark.parametrize("level", TERRAIN_LEVELS)
@pytest.mark.parametrize("name", TERRAIN_NAMES)
def test_height_query_matches_actual_mujoco_raycast(name, level):
    root, profile = scene(name, level=level,
                          difficulty=1.5 if level == "standard" else 1., seed=7)
    model, data = compile_scene(root)
    # Interior and just-outside samples, including either side of steps/ramps.
    boundaries = [profile.entry_x, profile.exit_x]
    if name in ("ramp", "stairs"):
        boundaries += [profile.parameters["ascent_end_x_m"],
                       profile.parameters["descent_start_x_m"]]
    xs = np.r_[np.linspace(.51, profile.finish_x + .21, 63),
               np.array(boundaries) - 1e-6, np.array(boundaries) + 1e-6]
    for x in xs:
        for y in (-1.21, -1.05, -.15, .15, 1.05, 1.21):
            assert profile.height_at(x, y) == pytest.approx(
                physical_height(model, data, x, y), abs=2e-10)


@pytest.mark.parametrize("name,x,y", [("flat", 2., 0.), ("ramp", 2., 0.),
                                        ("stairs", 2., 0.), ("rough", 1.2, -1.),
                                        ("hills", 2., 0.)])
def test_real_contact_supports_vertical_probe_at_the_declared_surface(name, x, y):
    root, profile = scene(name, seed=17)
    height, radius = profile.height_at(x, y), .04
    body = ET.SubElement(root.find("worldbody"), "body", name="probe",
                         pos=f"{x} {y} {height + .45}")
    # A vertical slider fixes XY, letting a sphere settle onto a slope without
    # rolling away from the location being checked.
    ET.SubElement(body, "joint", type="slide", axis="0 0 1")
    ET.SubElement(body, "geom", name="probe_ball", type="sphere",
                  size=str(radius), mass=".1")
    model, data = compile_scene(root)
    for _ in range(600):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    expected_z = height + (radius / math.cos(math.radians(8.)) if name == "ramp" else radius)
    assert data.xpos[model.body("probe").id, 2] == pytest.approx(expected_z, abs=1e-3)
    assert data.ncon > 0
    contacting_names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id))
                        for contact in data.contact for geom_id in (contact.geom1, contact.geom2)}
    assert "probe_ball" in contacting_names
    assert ("floor" in contacting_names if name == "flat"
            else any(name.startswith("terrain_") for name in contacting_names))


def test_difficulty_changes_vertical_features_without_shortening_the_route():
    for name in ("stairs", "rough"):
        _, easy = scene(name, difficulty=.5, seed=3)
        _, normal = scene(name, difficulty=1., seed=3)
        assert easy.finish_x == normal.finish_x
        x, y = np.meshgrid(np.linspace(1.03, 4.97, 71), [.13, .31])
        np.testing.assert_allclose(easy.height_at(x, y) * 2., normal.height_at(x, y), atol=1e-14)
    _, ramp = scene("ramp", difficulty=2.)
    assert ramp.parameters["slope_degrees"] == 16.
    assert ramp.height_at(2.) == pytest.approx(math.tan(math.radians(16.)))


def test_rough_seed_reproduces_geometry_and_changes_the_actual_surface():
    a, profile_a = scene("rough", seed=22)
    b, profile_b = scene("rough", seed=22)
    c, profile_c = scene("rough", seed=23)
    assert ET.tostring(a) == ET.tostring(b)
    assert profile_a.metadata == profile_b.metadata
    assert ET.tostring(a) != ET.tostring(c)
    assert profile_a.height_at(1.2, -.6) != profile_c.height_at(1.2, -.6)
    heights = np.array(profile_a.parameters["cell_heights_m"])
    assert np.all((0. <= heights) & (heights <= .025))


@pytest.mark.parametrize("level,amplitude", [("standard", .025), ("moderate", .05),
                                            ("hard", .08), ("extreme", .12)])
def test_hill_collision_surface_has_no_height_steps_at_any_segment_join(level, amplitude):
    root, profile = scene("hills", level=level)
    model, data = compile_scene(root)
    assert profile.metadata["parameters"]["height_is_continuous"] is True
    assert profile.metadata["parameters"]["slope_is_continuous"] is False
    assert profile.metadata["max_height_m"] == pytest.approx(amplitude)
    assert profile.height_at(2.) == pytest.approx(amplitude)
    assert profile.height_at(3.) == pytest.approx(0., abs=1e-12)
    # Check the actual raycasted surfaces on both sides of EVERY joined slab,
    # including the joins with the zero-height floor. A tiny change in x must
    # produce a tiny change in z; cell steps fail this observable contract.
    epsilon = 1e-8
    for x in profile.parameters["node_x_m"]:
        for y in (-.8, 0., .8):
            left, right = physical_height(model, data, x - epsilon, y), physical_height(model, data, x + epsilon, y)
            assert abs(left - right) < 2e-8
            assert profile.height_at(x - epsilon, y) == pytest.approx(left, abs=2e-10)
            assert profile.height_at(x + epsilon, y) == pytest.approx(right, abs=2e-10)


@pytest.mark.parametrize("kwargs", [{"name": "ice"}, {"difficulty": 0},
                                      {"difficulty": np.nan}, {"difficulty": 3.1},
                                      {"seed": -1}, {"seed": 1.2},
                                      {"level": "impossible"},
                                      {"level": "hard", "difficulty": 2.},
                                      {"level": "moderate", "difficulty": .5}])
def test_invalid_settings_do_not_change_the_scene(kwargs):
    root = ET.fromstring('<mujoco><worldbody/></mujoco>')
    before = ET.tostring(root)
    with pytest.raises(ValueError):
        append_terrain(root, **kwargs)
    assert ET.tostring(root) == before


def test_duplicate_course_and_nonfinite_height_queries_are_rejected():
    root, profile = scene("ramp")
    with pytest.raises(ValueError, match="already"):
        append_terrain(root, "stairs")
    with pytest.raises(ValueError, match="finite"):
        profile.height_at(np.nan, 0.)


@pytest.mark.skipif(os.environ.get("SIMPLE_RBT_TEST_WARP") != "1",
                    reason="Set SIMPLE_RBT_TEST_WARP=1 for host CPU/cuda:0 terrain comparison")
@pytest.mark.parametrize("level", ("standard", "extreme"))
@pytest.mark.parametrize("name", TERRAIN_NAMES)
def test_cpu_and_cuda_warp_rays_agree_on_physical_terrain(name, level):
    from mujoco_lidar import MjLidarWrapper
    import warp as wp

    root, profile = scene(name, level=level,
                          difficulty=1.5 if level == "standard" else 1., seed=31)
    origin = np.array([(profile.entry_x + profile.exit_x) / 2., 0., 5.])
    ET.SubElement(root.find("worldbody"), "site", name="lidar",
                  pos=" ".join(str(value) for value in origin))
    model, data = compile_scene(root)
    x, y = np.meshgrid(np.linspace(.53, profile.finish_x + .17, 19),
                       np.linspace(-1.33, 1.33, 13))
    targets = np.column_stack((x.ravel(), y.ravel(), profile.height_at(x, y).ravel()))
    vectors = targets - origin
    theta = np.arctan2(vectors[:, 1], vectors[:, 0]).astype(np.float32)
    phi = np.arctan2(vectors[:, 2], np.hypot(vectors[:, 0], vectors[:, 1])).astype(np.float32)
    wp.init()
    assert wp.get_device("cuda:0").is_cuda
    ranges = []
    for backend in ("cpu", "warp"):
        sensor = MjLidarWrapper(model, "lidar", backend=backend, cutoff_dist=30.,
                               args={"bodyexclude": -1, **({"device": "cuda:0"} if backend == "warp" else {})})
        ranges.append(np.asarray(sensor.trace_rays(data, theta, phi)).copy())
    wp.synchronize_device("cuda:0")
    assert np.isfinite(ranges[0]).all() and np.all(ranges[0] > 0.)
    np.testing.assert_allclose(ranges[1], ranges[0], atol=1e-4, rtol=0)


@pytest.mark.parametrize("name", TERRAIN_NAMES)
@pytest.mark.parametrize("difficulty", (.5, 1., 3.))
def test_explicit_standard_preserves_default_geometry(name, difficulty):
    implicit, a = scene(name, difficulty=difficulty, seed=22)
    explicit, b = scene(name, level="standard", difficulty=difficulty, seed=22)
    assert ET.tostring(implicit) == ET.tostring(explicit)
    assert a.metadata == b.metadata
    assert a.level == "standard"


@pytest.mark.parametrize("level,degrees,run,landing", [
    ("moderate", 15., 3., 1.), ("hard", 20., 4., 1.), ("extreme", 25., 4., 1.)])
def test_larger_ramps_have_complete_ascent_landing_descent(level, degrees, run, landing):
    _, profile = scene("ramp", level=level)
    peak = run * math.tan(math.radians(degrees))
    p = profile.parameters
    assert p["slope_degrees"] == degrees
    assert p["horizontal_run_m"] == run
    assert p["landing_length_m"] == landing
    assert profile.metadata["max_height_m"] == pytest.approx(peak)
    assert p["ascent_start_x_m"] == profile.entry_x == 1.
    assert p["ascent_end_x_m"] == p["landing_start_x_m"] == 1. + run
    assert p["landing_end_x_m"] == p["descent_start_x_m"] == 1. + run + landing
    assert p["descent_end_x_m"] == profile.exit_x == 1. + 2. * run + landing
    np.testing.assert_allclose(profile.height_at([
        1. + run / 2., 1. + run + landing / 2.,
        1. + 1.5 * run + landing]), [peak / 2., peak, peak / 2.], atol=1e-14)
    assert profile.height_at(profile.exit_x + .01) == 0.
    assert profile.finish_x == pytest.approx(profile.exit_x + .6)
    assert profile.finish_x < 12.


@pytest.mark.parametrize("level,rise,count,landing", [
    ("moderate", .12, 6, .9), ("hard", .18, 6, .9), ("extreme", .22, 8, 1.)])
def test_larger_stairs_have_every_riser_and_a_full_descent(level, rise, count, landing):
    _, profile = scene("stairs", level=level)
    tread = .3
    p = profile.parameters
    assert p["step_height_m"] == rise
    assert p["ascending_steps"] == p["descending_risers"] == count
    assert p["tread_length_m"] == tread
    assert p["landing_length_m"] == landing
    assert p["ascent_end_x_m"] == p["landing_start_x_m"] == 1. + count * tread
    assert p["landing_end_x_m"] == p["descent_start_x_m"] == 1. + count * tread + landing
    assert p["descent_end_x_m"] == profile.exit_x
    for index in range(count):
        assert profile.height_at(1. + (index + .5) * tread) == pytest.approx((index + 1) * rise)
    assert profile.height_at(p["landing_start_x_m"] + landing / 2.) == pytest.approx(count * rise)
    for index in range(count - 1):
        x = p["descent_start_x_m"] + (index + .5) * tread
        assert profile.height_at(x) == pytest.approx((count - index - 1) * rise)
    assert profile.height_at(profile.exit_x + 1e-6) == 0.
    assert profile.finish_x == pytest.approx(max(6., profile.exit_x + .6))
    assert profile.metadata["max_height_m"] == pytest.approx(count * rise)
    assert profile.finish_x < 12.


@pytest.mark.parametrize("level,maximum", [("moderate", .05), ("hard", .08), ("extreme", .12)])
def test_larger_rough_levels_use_the_same_seeded_pattern_with_larger_heights(level, maximum):
    _, standard = scene("rough", seed=22)
    _, profile = scene("rough", level=level, seed=22)
    standard_heights = np.array(standard.parameters["cell_heights_m"])
    heights = np.array(profile.parameters["cell_heights_m"])
    np.testing.assert_allclose(heights, standard_heights * maximum / .025)
    assert profile.parameters["maximum_cell_height_m"] == maximum
    assert profile.metadata["level"] == profile.level == level
    assert np.all((0. <= heights) & (heights <= maximum))
    assert profile.finish_x == 6.


@pytest.mark.parametrize("level", TERRAIN_LEVELS)
def test_flat_level_has_no_artificial_obstacles(level):
    root, profile = scene("flat", level=level)
    assert len(root.findall("./worldbody/geom")) == 1  # caller's floor only
    assert profile.metadata["geom_count"] == 0
    assert profile.level == level
    assert profile.finish_x == 6.
    assert profile.height_at(3.) == 0.


@pytest.mark.parametrize("name", ("ramp", "stairs"))
@pytest.mark.parametrize("level", ("moderate", "hard", "extreme"))
@pytest.mark.parametrize("part", ("ascent", "landing", "descent"))
def test_tall_course_contact_supports_probe_on_each_physical_section(name, level, part):
    root, profile = scene(name, level=level)
    p = profile.parameters
    begin, end = p[f"{part}_start_x_m"], p[f"{part}_end_x_m"]
    # Stay inside a tread rather than exactly at a stair edge.
    x = ((begin + end) / 2. if name == "ramp" or part == "landing"
         else begin + p["tread_length_m"] * .5)
    radius = .04
    surface = profile.height_at(x)
    body = ET.SubElement(root.find("worldbody"), "body", name="probe",
                         pos=f"{x} 0 {surface + .4}")
    ET.SubElement(body, "joint", type="slide", axis="0 0 1")
    ET.SubElement(body, "geom", name="probe_ball", type="sphere", size=str(radius), mass=".1")
    model, data = compile_scene(root)
    for _ in range(600):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    slope = math.radians(p["slope_degrees"]) if name == "ramp" and part != "landing" else 0.
    assert data.xpos[model.body("probe").id, 2] == pytest.approx(surface + radius / math.cos(slope), abs=1e-3)
    assert any(model.geom(int(contact.geom1)).name.startswith("terrain_")
               or model.geom(int(contact.geom2)).name.startswith("terrain_")
               for contact in data.contact)
