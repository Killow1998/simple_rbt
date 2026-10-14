"""Deployment phase integration and effective contact material behavior."""
import json
import numpy as np
import pytest

from simple_rbt_mujoco.core import Simulation


def test_getup_keeps_policy_history_and_tracker_idle_until_physics_completion():
    sim = Simulation("go2", controller="himloco", velocity_feedback=True,
                     go2_torque_mode="upstream", go2_startup="upstream_getup")
    limits = sim.model.actuator_ctrlrange.copy()
    sim.set_command(.6, .1, .2)
    while not sim.startup_complete:
        sim.step()
        assert sim.controller.steps == 0
        assert sim.command_tracker.update_count == 0
        np.testing.assert_array_equal(sim.controller.history, 0.)
        np.testing.assert_array_equal(sim.policy_command, 0.)
        np.testing.assert_array_equal(sim.data.qfrc_applied, 0.)
        np.testing.assert_array_equal(sim.data.xfrc_applied, 0.)
    assert sim.startup_steps == sim.steps == 601
    assert sim.data.time == pytest.approx(sim.startup_elapsed_s)
    assert sim.stamp_ns == 3_005_000_000
    assert sim.startup_phase == "policy"
    assert sim.pose()[0][2] > .2
    sim.step()
    assert sim.controller.steps == 1
    assert sim.command_tracker.update_count == 1
    assert sim.stamp_ns == 3_010_000_000
    np.testing.assert_array_equal(sim.model.actuator_ctrlrange, limits)


@pytest.mark.parametrize("robot,default_mu", [("go2", .4), ("go2w", .8)])
@pytest.mark.parametrize("coefficient", [None, .2, .4, .8])
def test_requested_surface_friction_controls_real_wheel_or_foot_contacts(robot, default_mu, coefficient):
    sim = Simulation(robot, terrain_friction=coefficient)
    seen = []
    for _ in range(100):
        sim.step()
        for contact in sim.data.contact:
            ids = [int(g) for g in (contact.geom1, contact.geom2)]
            names = [sim.model.geom(g).name for g in ids]
            if "floor" in names and any(sim.model.geom_priority[g] == 1 for g in ids):
                seen.append(contact.friction.copy())
    assert seen, "A real robot-ground contact is required to verify the material"
    expected = default_mu if coefficient is None else coefficient
    np.testing.assert_allclose(np.asarray(seen)[:, :2], expected, atol=1e-12)
    rotation_friction = np.asarray(seen)[:, 2:]
    np.testing.assert_allclose(rotation_friction,
                               np.broadcast_to([.02, .01, .01], rotation_friction.shape), atol=1e-12)
    assert sim.terrain.metadata["friction"][0] == (1. if coefficient is None else coefficient)
    assert ("friction_override" in sim.terrain.parameters) == (coefficient is not None)


@pytest.mark.parametrize("coefficient", [0., -.1, 2.1, float("nan"), float("inf")])
def test_invalid_friction_is_rejected(coefficient):
    with pytest.raises(ValueError, match="friction"):
        Simulation("go2w", terrain_friction=coefficient)


@pytest.mark.parametrize("robot,controller", [("go2w", "auto"), ("diffdrive", "auto"), ("go2", "legacy")])
def test_go2_deployment_choices_cannot_silently_apply_to_other_controllers(robot, controller):
    with pytest.raises(ValueError, match="alternatives require"):
        Simulation(robot, controller=controller, go2_torque_mode="upstream")
    with pytest.raises(ValueError, match="alternatives require"):
        Simulation(robot, controller=controller, go2_startup="upstream_getup")


@pytest.mark.parametrize("duration,started", [(4., False), (6., True)])
def test_cli_reports_actual_command_start_after_getup_and_settle(tmp_path, capsys, duration, started):
    from simple_rbt_mujoco.cli import main
    output = tmp_path / "timing"
    main(["--robot", "go2", "--controller", "robot_lab", "--go2-startup", "upstream_getup",
          "--sensors", "physics", "--summary-only", "--cmd", ".3", "0", "0",
          "--settle", "2", "--duration", str(duration), "--output", str(output)])
    capsys.readouterr()
    result = json.loads((output / "summary.json").read_text())
    assert result["simulation_seconds"] == pytest.approx(duration)
    assert result["policy_clock_origin_s"] == pytest.approx(3.005)
    planned = result["planned_movement_command_start_s"]
    assert planned == pytest.approx(5.005)
    if started:
        assert planned <= result["movement_command_start_s"] <= planned + .005 + 1e-9
        assert result["movement_command_start_s"] < result["simulation_seconds"]
    else:
        assert planned > result["simulation_seconds"]
        assert result["movement_command_start_s"] is None
