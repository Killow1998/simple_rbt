import numpy as np
import pytest

from simple_rbt_mujoco.core import Simulation, UPSTREAM


@pytest.mark.parametrize("robot", ["diffdrive", "go2"])
def test_forward_motion_remains_upright(robot):
    if robot == "go2" and not UPSTREAM.exists():
        pytest.skip("Run setup.sh for upstream Go2 assets")
    sim = Simulation(robot)
    start = sim.pose()[0]
    steps = round(5. / sim.model.opt.timestep)
    settle_steps = round(1. / sim.model.opt.timestep)
    for i in range(steps):
        sim.set_command(.3 if i >= settle_steps else 0., 0., 0.)
        sim.step()
        assert sim.pose()[0][2] > (.20 if robot == "go2" else .10)
        assert sim.data.site_xmat[sim.model.site("base_origin").id].reshape(3, 3)[2, 2] > .9
    assert sim.pose()[0][0] - start[0] > .5
    assert sim.stamp_ns == 5_000_000_000
    assert np.isfinite(np.concatenate(sim.imu())).all()


def test_stationary_imu_specific_force_and_command_validation():
    sim = Simulation("diffdrive")
    for _ in range(500):
        sim.step()
    gyro, force, quat = sim.imu()
    np.testing.assert_allclose(force, [0., 0., 9.81], atol=.1)
    assert np.linalg.norm(gyro) < .02
    assert np.linalg.norm(quat) == pytest.approx(1.)
    with pytest.raises(ValueError):
        sim.set_command(np.nan, 0, 0)
    with pytest.raises(ValueError):
        sim.set_command(0, 1, 0)


def test_diffdrive_turns_left_for_positive_yaw_command():
    sim = Simulation("diffdrive")
    sim.set_command(0., 0., .4)
    for _ in range(750):
        sim.step()
    quat = sim.pose()[1]
    yaw = 2 * np.arctan2(quat[2], quat[3])
    assert .5 < yaw < 1.5
