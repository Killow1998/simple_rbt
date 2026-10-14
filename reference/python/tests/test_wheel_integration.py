"""Shared-core wheel command contracts and real physics integration."""
import numpy as np
import pytest

from simple_rbt_mujoco.core import Simulation
from simple_rbt_mujoco.robots import DEFAULT_CONTROLLERS, WHEELED_ROBOTS, WHEEL_MODELS


@pytest.mark.parametrize("robot", WHEEL_MODELS)
@pytest.mark.parametrize("tracked", [False, True])
def test_auto_wheel_controller_drives_real_body_with_optional_tracker(robot, tracked):
    sim = Simulation(robot, velocity_feedback=tracked)
    assert sim.controller_name == DEFAULT_CONTROLLERS[robot]
    sim.set_command(.3, 0., 0.)
    for _ in range(round(1. / sim.model.opt.timestep)):
        sim.step()
    assert sim.pose()[0][0] > .2
    assert .2 < sim.velocity()[0][0] < .4
    assert np.isfinite(sim.data.qpos).all()


@pytest.mark.parametrize("robot", WHEELED_ROBOTS)
@pytest.mark.parametrize("surface", ["stairs", "rough"])
def test_step_surfaces_are_rejected_before_building_a_wheel_robot(robot, surface):
    with pytest.raises(ValueError, match="continuous surfaces"):
        Simulation(robot, terrain=surface)


@pytest.mark.parametrize("requested_velocity", [(0., 0., .2), (1e-12, 0., .2),
                                     (.3, .1, 0.), (np.nan, 0., 0.), ("bad", 0., 0.),
                                     ([.3], [0.], [0.])])
def test_rejected_direct_command_clears_previous_fr_movement(requested_velocity):
    sim = Simulation("fr_mini")
    sim.set_command(.3, 0., .1)
    with pytest.raises(ValueError):
        sim.set_command(*requested_velocity)
    np.testing.assert_array_equal(sim.command, [0., 0., 0.])
    sim.step()  # A caller that handles the exception can safely continue.
    np.testing.assert_array_equal(sim.policy_command, [0., 0., 0.])
    sim.set_command(-.3, 0., .1)
    assert sim.command[0] == -.3
