"""Scout acceptance gates operate on observable motion and physical contacts."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/evaluate_scout_mini.py"
SPEC = importlib.util.spec_from_file_location("scout_evaluation", SCRIPT)
evaluation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluation)


def test_case_plan_keeps_explicit_targets_and_does_not_silently_change_limits():
    cases = evaluation.make_cases()
    assert len(cases) == 9
    assert [c["command_vx_vy_wz"][0] for c in cases[:4]] == [.3, .6, 1., -.3]
    assert {c["terrain"] for c in cases} == {"flat", "ramp", "hills"}
    assert all(c["terrain_level"] == "standard" for c in cases)
    assert evaluation.make_cases(["turn_left", "turn_left", "hills"])[0]["command_vx_vy_wz"] == [0., 0., .5]
    with pytest.raises(ValueError, match="Unknown case"):
        evaluation.make_cases(["pretend_success"])


def measurements(vx=.6, wz=0.):
    t = np.arange(.005, 6.001, .005)
    velocity = np.zeros((len(t), 6))
    velocity[:, 0], velocity[:, 5] = vx, wz
    return t, velocity, t * wz


def test_speed_gate_uses_real_body_velocity_with_signed_reverse_and_transient_rmse():
    t, velocity, yaw = measurements()
    good = evaluation.tracking_metrics(t, velocity, yaw, [.6, 0., 0.])
    assert good["passed"]
    assert good["thresholds"]["mean_vx_error_m_s"] == pytest.approx(.09)
    velocity[:, 0] = .4
    assert not evaluation.tracking_metrics(t, velocity, yaw, [.6, 0., 0.])["passed"]
    velocity[:, 0] = -.3
    assert evaluation.tracking_metrics(t, velocity, yaw, [-.3, 0., 0.])["passed"]
    # Startup lag remains visible in RMSE even though steady tracking succeeds.
    velocity[t < 1., 0] = 0.
    result = evaluation.tracking_metrics(t, velocity, yaw, [-.3, 0., 0.])
    assert result["passed"] and result["full_motion_rmse_vx_vy_wz"][0] > .1


def test_zero_mean_oscillation_does_not_hide_failed_heading_or_parking():
    t, velocity, yaw = measurements(vx=0.)
    velocity[:, 5] = np.where(np.arange(len(t)) % 2, .2, -.2)
    result = evaluation.tracking_metrics(t, velocity, yaw, [0., 0., 0.])
    assert result["checks"]["mean_wz_error"]
    assert not result["checks"]["zero_yaw_oscillation"]
    velocity[:, 5] = 0.
    velocity[:, 0] = np.where(np.arange(len(t)) % 2, .1, -.1)
    assert not evaluation.parking_metrics(t[t <= 2.], velocity[t <= 2.])["passed"]


def test_world_heading_rate_is_separate_from_body_yaw_rate():
    t, velocity, _ = measurements(vx=0., wz=.5)
    result = evaluation.tracking_metrics(t, velocity, t * .55 + 3., [0., 0., .5])
    assert result["passed"]
    assert result["steady_mean_body_vx_vy_vz_wx_wy_wz"][5] == pytest.approx(.5)
    assert result["steady_world_heading_rate_rad_s"] == pytest.approx(.55)
    assert not evaluation.tracking_metrics(t[:100], velocity[:100], t[:100], [0., 0., .5])["passed"]


def scout_fixture():
    from simple_rbt_mujoco.scout_mini import build_scout_mini_xml
    root = ET.fromstring(build_scout_mini_xml())
    ET.SubElement(root.find("worldbody"), "geom", name="floor", type="plane", size="10 10 .1")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_compiled_scout_clearance_and_wheel_contacts_are_not_go2_base_height_gate():
    model, data = scout_fixture()
    geometry = evaluation.geometry_contract(model, data)
    assert geometry["nominal_chassis_bottom_world_z_m"] == pytest.approx(.115)
    assert geometry["outer_half_width_m"] == pytest.approx(.290)
    wheel_ids = [model.geom(name + "_tire").id for name in geometry["wheel_names"]]
    # Tire contact is part of normal operation and does not count as body impact.
    data.qpos[2] -= .001
    mujoco.mj_forward(model, data)
    assert data.ncon > 0
    assert evaluation.body_environment_contacts(model, data, wheel_ids) == []
    # Move the same collision model down until its chassis penetrates the floor.
    data.qpos[2] -= .12
    mujoco.mj_forward(model, data)
    assert any("chassis" in name for name in evaluation.body_environment_contacts(model, data, wheel_ids))


@pytest.mark.parametrize("case", ["stairs_4cm", "stairs_5cm", "stairs", "rough"])
def test_discontinuous_surfaces_are_excluded_from_wheel_acceptance(case):
    with pytest.raises(ValueError, match="Unknown case"):
        evaluation.make_cases([case])
    if case in ("stairs", "rough"):
        from simple_rbt_mujoco.core import Simulation
        with pytest.raises(ValueError, match="continuous surfaces"):
            Simulation("scout_mini", terrain=case)


def test_rotated_ellipsoid_tire_envelope_uses_ellipsoid_support_not_box_bound():
    from simple_rbt_mujoco.scout_mini import build_scout_mini_xml
    root = ET.fromstring(build_scout_mini_xml())
    # A deliberately anisotropic, rotated fixture distinguishes exact support
    # from the larger rotated bounding-box approximation. This is not a drive run.
    for geom in root.findall(".//geom"):
        if geom.get("name", "").endswith("_tire"):
            geom.set("type", "ellipsoid")
            geom.set("size", ".08 .03 .05")
            geom.set("quat", f"{np.cos(np.pi / 8)} 0 0 {np.sin(np.pi / 8)}")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    geometry = evaluation.geometry_contract(model, data)
    expected_half_width = .490 / 2. + np.sqrt((.08 ** 2 + .03 ** 2) / 2.)
    box_half_width = .490 / 2. + (.08 + .03) / np.sqrt(2.)
    assert geometry["outer_half_width_m"] == pytest.approx(expected_half_width)
    assert geometry["outer_half_width_m"] < box_half_width - .01
    assert geometry["wheel_radius_m"] == [.08] * 4


def test_rated_maximum_speed_is_an_explicit_optional_case():
    original_cases = evaluation.make_cases()
    assert len(original_cases) == 9
    assert all(c["case"] != "forward_1.5" for c in original_cases)
    case = evaluation.make_cases(["forward_1.5"])[0]
    assert case["terrain"] == "flat"
    assert case["command_vx_vy_wz"] == [1.5, 0., 0.]
    sim = evaluation.build_case_simulation(case)
    sim.set_command(*case["command_vx_vy_wz"])
    np.testing.assert_array_equal(sim.command, [1.5, 0., 0.])
    assert case["settle_s"] == 2. and case["flat_motion_s"] == 6. and case["stop_s"] == 2.
