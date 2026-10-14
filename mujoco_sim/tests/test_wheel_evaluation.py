"""Robot-aware gates and contact telemetry for the unified wheel evaluator."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/evaluate_wheels.py"
SPEC = importlib.util.spec_from_file_location("wheel_evaluation", SCRIPT)
evaluation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluation)


def test_default_plan_has_only_supported_commands_and_continuous_surfaces():
    from simple_rbt_mujoco.robots import command_is_supported
    cases = evaluation.make_cases()
    assert len(cases) == 81
    assert len({case["case"] for case in cases}) == 81
    assert {case["terrain"] for case in cases} == {"flat", "ramp", "hills"}
    assert all(command_is_supported(case["robot"], *case["command_vx_vy_wz"]) for case in cases)
    counts = {robot: sum(case["robot"] == robot for case in cases) for robot in evaluation.ROBOTS}
    assert counts == {"scout_mini": 19, "fr_mini": 18, "ridgeback": 22, "ranger_mini": 22}
    fr = [case for case in cases if case["robot"] == "fr_mini" and case["terrain"] == "flat"]
    assert {case["motion"] for case in fr} == {
        "forward_0.3", "forward_0.6", "forward_1.0", "reverse_0.3", "arc_left", "reverse_arc_left"}
    assert len(evaluation.make_cases(control_modes=("native", "tracked"))) == 162
    assert all(case["settle_s"] == 2. and case["stop_s"] == 2. for case in cases)


@pytest.mark.parametrize("kwargs", [{"speeds": [0.]}, {"speeds": [float("nan")]},
                                     {"speeds": [1.5]}, {"levels": ["fake"]},
                                     {"robots": ["go2"]}, {"speeds": [.3, .3]}])
def test_invalid_sweeps_fail_before_creating_results(kwargs):
    with pytest.raises(ValueError):
        evaluation.make_cases(**kwargs)


def measurements(vx=0., vy=0., wz=0.):
    stamps = np.arange(.005, 6.001, .005)
    velocities = np.zeros((len(stamps), 6))
    velocities[:, 0], velocities[:, 1], velocities[:, 5] = vx, vy, wz
    return stamps, velocities, stamps * wz


@pytest.mark.parametrize("vy", [.3, -.3])
def test_commanded_lateral_speed_has_error_gate_not_scout_zero_lateral_gate(vy):
    stamps, values, yaw = measurements(vy=vy)
    result = evaluation.tracking_metrics(stamps, values, yaw, [0., vy, 0.])
    assert result["passed"]
    assert result["thresholds"]["mean_vy_error_m_s"] == .05
    assert "mean_absolute_vy" not in result["checks"]
    values[:, 1] = -vy
    assert not evaluation.tracking_metrics(stamps, values, yaw, [0., vy, 0.])["passed"]
    # Zero-target oscillation must not pass merely because its signed mean is 0.
    values[:, 1] = np.where(np.arange(len(stamps)) % 2, .2, -.2)
    result = evaluation.tracking_metrics(stamps, values, yaw, [0., 0., 0.])
    assert not result["passed"]
    assert result["thresholds"]["mean_absolute_vy_m_s"] == .1


def test_compound_velocity_and_stop_gates_keep_original_numeric_thresholds():
    stamps, values, yaw = measurements(.3, .2, .3)
    result = evaluation.tracking_metrics(stamps, values, yaw, [.3, .2, .3])
    assert result["passed"]
    assert result["thresholds"]["mean_vx_error_m_s"] == .05
    assert result["thresholds"]["mean_wz_error_rad_s"] == .05
    values[:, 0] = .2
    assert not evaluation.tracking_metrics(stamps, values, yaw, [.3, .2, .3])["passed"]
    values.fill(0.); values[:, 1] = .06
    assert not evaluation.parking_metrics(stamps[stamps <= 2.], values[stamps <= 2.])["passed"]
    values.fill(0.)
    assert evaluation.parking_metrics(stamps[stamps <= 2.], values[stamps <= 2.])["passed"]


def test_longer_routes_get_distance_budget_without_relaxing_motion_gates():
    case = next(case for case in evaluation.make_cases(robots=("fr_mini",), speeds=(.3,), levels=("moderate",))
                if case["terrain"] == "ramp")
    assert evaluation.motion_budget(case, SimpleNamespace(finish_x=6.)) == pytest.approx(8. / .3)
    assert evaluation.motion_budget(case, SimpleNamespace(finish_x=8.6)) == pytest.approx(10.6 / .3)
    case["command_vx_vy_wz"][0] = 0.
    with pytest.raises(ValueError, match="positive"):
        evaluation.motion_budget(case, SimpleNamespace(finish_x=6.))


def test_fr_telemetry_distinguishes_passive_fronts_and_uses_offset_chassis_pose():
    from simple_rbt_mujoco.core import Simulation
    sim = Simulation("fr_mini")
    sim.set_command(.3, 0., .2)
    for _ in range(4):
        sim.step()
    state = evaluation.controller_state(sim.controller, sim.model, sim.data)
    assert len(state["wheel_position_rad"]) == 4
    assert len(state["wheel_target_rad_s"]) == 4
    assert state["drive_actuator_names"] == ["rear_left_wheel", "rear_right_wheel"]
    assert len(state["steering_position_rad"]) == 2
    # Artificial reference surface varies with X, making a false x=0 chassis
    # assumption observable even before traversing any actual course.
    surface = SimpleNamespace(height_at=lambda x, y: .02 * np.asarray(x))
    gid = sim.model.geom("chassis").id
    centre = sim.data.geom_xpos[gid]
    rotation = sim.data.geom_xmat[gid].reshape(3, 3)
    size = sim.model.geom_size[gid]
    points = np.array([[x * size[0], y * size[1], -size[2]] for x in (-1, 0, 1) for y in (-1, 0, 1)])
    points = points @ rotation.T + centre
    expected = float(np.min(points[:, 2] - .02 * points[:, 0]))
    assert evaluation.chassis_bottom_gap(sim.model, sim.data, surface) == pytest.approx(expected)


def test_short_actual_fr_case_records_measured_motion_and_sources():
    case = next(c for c in evaluation.make_cases(robots=("fr_mini",), speeds=(.3,), levels=("standard",))
                if c["motion"] == "arc_left" and c["terrain"] == "flat")
    case["flat_motion_s"] = 2.25
    result = evaluation.evaluate_case(case)
    assert result["passed"]
    assert result["actual_controller"] == "ackermann"
    assert not result["control_contract"]["additional_body_tracker"]
    assert len(result["actuation"]["drive_actuator_names"]) == 2
    hashes = result["provenance"]["source_sha256"]
    assert "mujoco_sim/simple_rbt_mujoco/wheel_geometry.py" in hashes
    assert "mujoco_sim/scripts/evaluate_scout_mini.py" in hashes
    assert "mujoco_sim/assets/fr_mini_manifest.json" in hashes
    assert result["non_wheel_contact_names"] == []
    json.dumps(result, allow_nan=False)


def test_nonfinite_simulation_failure_keeps_last_finite_clock_and_json(monkeypatch):
    import simple_rbt_mujoco.core as core
    sim = core.Simulation("fr_mini")
    def broken_step():
        sim.data.time = float("nan")
        sim.data.qpos[0] = float("nan")
        raise RuntimeError("synthetic nonfinite physics state")
    sim.step = broken_step
    monkeypatch.setattr(core, "Simulation", lambda *args, **kwargs: sim)
    case = evaluation.make_cases(robots=("fr_mini",), speeds=(.3,), levels=("standard",))[0]
    result = evaluation.evaluate_case(case)
    assert not result["passed"]
    assert result["failure_reason"].startswith("simulation_error")
    assert result["last_finite_stamp_ns"] == 0
    assert result["last_finite_simulation_time_s"] == 0.
    assert not result["parking_evaluated"]
    json.dumps(result, allow_nan=False)


def test_rotated_rear_axle_origin_chassis_uses_world_width_for_route_gate(monkeypatch):
    import mujoco
    import simple_rbt_mujoco.core as core
    from simple_rbt_mujoco.wheel_geometry import collision_envelope

    sim = core.Simulation("fr_mini", terrain="ramp")
    # A 45-degree car's front-right corner extends farther in world Y than
    # body-frame half-width; FR's reference is at the rear axle, not its centre.
    # Start clear of the terrain/walls so the first observed failure is the
    # explicit 0.1 m corridor margin rather than a body collision.
    sim.data.qpos[:3] = [sim.terrain.entry_x + .05, .7, .5]
    sim.data.qpos[3:7] = [np.cos(np.pi / 8), 0., 0., np.sin(np.pi / 8)]
    mujoco.mj_forward(sim.model, sim.data)
    body = collision_envelope(sim.model, sim.data)
    world = collision_envelope(sim.model, sim.data, reference_frame="world")
    assert body["outer_half_width_m"] == pytest.approx(.2025)
    assert world["outer_half_width_m"] == pytest.approx((.5075 + .2025) / np.sqrt(2.))
    old_margin = sim.terrain.width / 2. - body["outer_half_width_m"] - .1
    corrected_margin = sim.terrain.width / 2. - world["outer_half_width_m"] - .1
    assert corrected_margin < sim.data.qpos[1] < old_margin
    monkeypatch.setattr(core, "Simulation", lambda *args, **kwargs: sim)
    case = next(c for c in evaluation.make_cases(robots=("fr_mini",), speeds=(.3,), levels=("standard",))
                if c["terrain"] == "ramp")
    case["settle_s"] = 0.
    result = evaluation.evaluate_case(case)
    assert result["failure_reason"] == "course_corridor_exit"
    assert result["non_wheel_contact_names"] == []
    assert result["compiled_geometry_initial_world"]["reference_frame"] == "world"
    assert result["minimum_corridor_half_width_m"] == pytest.approx(corrected_margin, abs=1e-6)
    assert result["last_finite_simulation_time_s"] == pytest.approx(sim.model.opt.timestep)
