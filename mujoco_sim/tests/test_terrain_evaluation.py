"""Route/evaluator contracts without policy inference or long physics runs."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/evaluate_terrains.py"
SPEC = importlib.util.spec_from_file_location("terrain_evaluator", SCRIPT)
evaluation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluation)


def profile(finish=10.6, level="extreme"):
    parameters = {"ascent_start_x_m": 1., "ascent_end_x_m": 5.,
                  "landing_start_x_m": 5., "landing_end_x_m": 6.,
                  "descent_start_x_m": 6., "descent_end_x_m": 10.}
    return SimpleNamespace(name="ramp", level=level, entry_x=1., exit_x=10.,
                           finish_x=finish, width=2.4, parameters=parameters,
                           metadata={"name": "ramp", "level": level}, height_at=lambda x, y=0.: 0.)


def config(**changes):
    values = {"robot": "go2", "controller": "robot_lab", "terrain": "ramp",
              "terrain_level": "extreme", "difficulty": 1., "seed": 0,
              "control_mode": "native", "speed_m_s": 1., "settle_s": 0.,
              "commanded_distance_budget_m": 8., "max_samples": 200}
    values.update(changes)
    return values


def test_long_course_deadline_extends_without_reducing_standard_allowance():
    standard = profile(finish=6., level="standard")
    budget, deadline = evaluation.route_budget(config(speed_m_s=.3, settle_s=2.), standard)
    assert budget == 8.
    assert deadline == pytest.approx(2. + 8. / .3)
    budget, deadline = evaluation.route_budget(config(speed_m_s=.3, settle_s=2.), profile())
    assert budget == 12.6
    assert deadline == pytest.approx(44.)
    assert evaluation.route_budget(config(commanded_distance_budget_m=15.), profile())[0] == 15.


def test_progress_tracks_actual_ascent_top_descent_and_exit_boundaries():
    route = profile()
    assert [evaluation.course_phase(route, x) for x in (.9, 1., 4.99, 5., 6., 9.99, 10., 10.6)] == [
        "approach", "ascent", "ascent", "top", "descent", "descent", "exit", "complete"]
    # The old X=6 completion point is now the beginning of this route's descent.
    assert evaluation.course_phase(route, 6.) != "complete"


def install_fake_simulation(monkeypatch, *, nonfinite=False, stop_x=None, terrain=None,
                            getup_steps=0, getup_contact=False, getup_low=False,
                            bad_pose_persists=False, getup_tilt=False, never_ready=False,
                            timestep=.1):
    from simple_rbt_mujoco import core
    seen = []

    class FakeSimulation:
        def __init__(self, robot, **kwargs):
            seen.append(kwargs)
            self.terrain = terrain or profile(level=kwargs["terrain_level"])
            self.model = SimpleNamespace(opt=SimpleNamespace(timestep=timestep),
                                         site_bodyid=np.array([1]), site=lambda name: SimpleNamespace(id=0),
                                         geom_bodyid=np.array([0, 1]), geom=lambda idx: SimpleNamespace(name="torso"))
            self.data = SimpleNamespace(time=0., site_xmat=np.eye(3).reshape(1, 9), contact=[])
            self.position = np.array([0., 0., .5])
            self.command = np.zeros(3)
            self.policy_command = np.zeros(3)
            self.command_tracker = None
            self.controller_name = kwargs["controller"]
            self.steps = 0
            self.command_calls = []
            kwargs["_command_calls"] = self.command_calls
            self.deployment_metadata = {"go2_startup": kwargs.get("go2_startup"),
                                        "go2_torque_mode": kwargs.get("go2_torque_mode")}

        @property
        def startup_complete(self):
            return not never_ready and self.steps >= getup_steps

        @property
        def startup_steps(self):
            return self.steps if never_ready else min(self.steps, getup_steps)

        @property
        def startup_elapsed_s(self):
            return self.startup_steps * timestep

        @property
        def startup_phase(self):
            return "policy" if self.startup_complete else "getup"

        @property
        def stamp_ns(self):
            return round(self.steps * timestep * 1e9)

        def pose(self):
            return self.position.copy(), np.array([0., 0., 0., 1.])

        def set_command(self, *command):
            self.command_calls.append((self.data.time, command))
            self.command[:] = command
            self.policy_command[:] = command

        def velocity(self):
            return np.array([self.command[0], self.command[1], 0.]), np.zeros(3)

        def step(self):
            was_startup = not self.startup_complete
            self.steps += 1
            self.data.time += timestep
            if was_startup:
                self.position[0] += .001
            elif stop_x is None or self.position[0] < stop_x:
                self.position[0] += self.command[0] * timestep
            low_or_contact = was_startup or bad_pose_persists
            self.position[2] = .05 if getup_low and low_or_contact else .5
            self.data.contact = [SimpleNamespace(geom1=0, geom2=1)] if getup_contact and low_or_contact else []
            if getup_tilt and was_startup:
                angle = np.radians(60.)
                self.data.site_xmat[:] = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],
                                                  [-np.sin(angle), 0, np.cos(angle)]]).reshape(1, 9)
            if nonfinite and self.steps == 2:
                self.position[0] = np.nan
                self.data.time = np.nan

    monkeypatch.setattr(core, "Simulation", FakeSimulation)
    return seen


def test_long_course_completes_after_old_deadline_and_dispatches_level(monkeypatch):
    seen = install_fake_simulation(monkeypatch)
    result = evaluation.evaluate_case(config())
    assert seen[0]["terrain_level"] == "extreme"
    assert result["success"]
    assert result["simulation_seconds"] > 8.
    assert result["thresholds"]["maximum_simulation_duration_s"] == 12.6
    assert result["progress"]["final_phase"] == "complete"
    assert [row["phase"] for row in result["progress"]["phase_transitions"]] == [
        "approach", "ascent", "top", "descent", "exit", "complete"]
    assert result["terrain_level"] == "extreme"


def test_timeout_reports_the_actual_failed_phase(monkeypatch):
    install_fake_simulation(monkeypatch, stop_x=7.)
    result = evaluation.evaluate_case(config())
    assert not result["success"]
    assert result["failure_reason"] == "time_limit_before_finish"
    assert result["progress"]["failure_phase"] == "descent"
    assert result["progress"]["furthest_phase"] == "descent"


def test_nonfinite_state_preserves_last_finite_pose_and_json_report(monkeypatch):
    install_fake_simulation(monkeypatch, nonfinite=True)
    result = evaluation.evaluate_case(config())
    assert result["failure_reason"] == "nonfinite_state"
    assert result["final_position_m"] == [.1, 0., .5]
    assert result["simulation_seconds"] is None
    assert result["last_finite_simulation_seconds"] == .1
    assert result["loop_real_time_factor"] is None
    json.dumps(result, allow_nan=False)


def stair_fixture():
    # A deliberately tall step: the base centre can fail the 10 cm margin while
    # its 10 cm thick torso still remains above the step without any contact.
    return SimpleNamespace(name="stairs", entry_x=0., exit_x=1., finish_x=2., width=2.4,
                           parameters={"ascending_steps": 1, "descending_risers": 1,
                                       "tread_length_m": .5, "ascent_start_x_m": 0.,
                                       "ascent_end_x_m": .5, "landing_end_x_m": 1.,
                                       "descent_start_x_m": 1., "descent_end_x_m": 1.},
                           metadata={"name": "stairs", "level": "extreme"},
                           height_at=lambda x, y=0.: .3 if 0. <= x <= 1. else 0.)


def test_tall_riser_clearance_can_fail_without_physical_torso_contact():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom type="box" pos=".5 0 .15" size=".5 1 .15"/>
      <body pos="0 0 .39"><freejoint/><geom type="box" size=".03 .03 .05" mass="1"/></body>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.ncon == 0
    observation = evaluation.clearance_observation(stair_fixture(), [0., 0., .39], [-.01, 0., .39], False)
    assert observation["base_clearance_m"] == pytest.approx(.09)
    assert observation["at_vertical_height_discontinuity"]
    assert observation["height_before_after_10um_m"] == [0., .3]
    assert observation["crossed_stair_riser_x_m"] == [0.]
    assert not observation["torso_contact"]


def test_low_clearance_keeps_safety_gate_without_claiming_observed_fall(monkeypatch):
    stairs = stair_fixture()
    stairs.height_at = lambda x, y=0.: .45 if 0. <= x <= 1. else 0.
    install_fake_simulation(monkeypatch, terrain=stairs)
    result = evaluation.evaluate_case(config(terrain="stairs"))
    assert result["failure_reason"] == "base_clearance_limit"
    assert result["clearance_gate_triggered"] and result["safety_gate_triggered"]
    assert result["fall_detected"]  # Historical field explicitly retains its old broad meaning.
    assert not result["actual_tilt_or_contact_failure"]
    assert not result["clearance_observation"]["torso_contact"]
    assert result["progress"]["failure_phase"] == "ascent"


def test_cli_generates_distinct_level_cases_with_robot_lab_default(tmp_path, monkeypatch):
    configs = []

    def worker(command, **kwargs):
        if command[0] == "git":
            return SimpleNamespace(stdout="fixture-base-commit\n", returncode=0)
        case = json.loads(command[command.index("--case-json") + 1])
        configs.append(case)
        path = Path(command[command.index("--case-output") + 1])
        path.write_text(json.dumps({"case": case, "success": False,
                                    "failure_reason": "time_limit_before_finish"}))
        return SimpleNamespace(stdout="", returncode=0)

    monkeypatch.setattr(evaluation.subprocess, "run", worker)
    output = tmp_path / "new-cases"
    assert evaluation.main(["--robots", "go2", "--terrains", "ramp", "--speeds", "1",
                            "--levels", "standard", "extreme", "--output", str(output)]) == 0
    assert [case["terrain_level"] for case in configs] == ["standard", "extreme"]
    assert all(case["controller"] == "robot_lab" for case in configs)
    names = [p.name for p in (output / "cases").glob("*.json")]
    assert any("-ramp-standard-" in name for name in names)
    assert any("-ramp-extreme-" in name for name in names)


def test_getup_completion_starts_fresh_settle_and_full_route_budget(monkeypatch):
    seen = install_fake_simulation(monkeypatch, getup_steps=601, getup_contact=True,
                                   getup_low=True, stop_x=.601, timestep=.005)
    result = evaluation.evaluate_case(config(go2_startup="upstream_getup",
                                              go2_torque_mode="upstream", settle_s=2.))
    assert seen[0]["go2_startup"] == "upstream_getup"
    assert seen[0]["go2_torque_mode"] == "upstream"
    assert result["startup"]["steps"] == 601
    assert result["startup"]["elapsed_s"] == pytest.approx(3.005)
    assert result["startup"]["absolute_end_sim_s"] == pytest.approx(3.005)
    assert result["startup"]["displacement_world_m"][0] == pytest.approx(.601)
    assert result["startup"]["torso_contact_steps"] == 601
    assert result["startup"]["minimum_base_clearance_m"] == .05
    assert result["torso_contact_steps"] == 0
    assert result["minimum_base_clearance_m"] == .5
    assert result["failure_reason"] == "time_limit_before_finish"
    assert result["timing"]["evaluation_elapsed_s"] == pytest.approx(14.6)
    assert result["timing"]["absolute_deadline_sim_s"] == pytest.approx(17.605)
    assert result["simulation_seconds"] == pytest.approx(17.605)
    commands = seen[0]["_command_calls"]
    assert all(command == (0., 0., 0.) for stamp, command in commands if stamp < 5.005)
    assert 2. <= result["timing"]["actual_settle_elapsed_s"] <= 2.005000001
    assert result["startup_resources"] is not None
    assert result["post_startup_resources"] is not None
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("low,contact,reason", [
    (True, False, "base_clearance_limit"), (False, True, "torso_environment_contact")])
def test_persistent_startup_pose_fails_on_first_policy_step(monkeypatch, low, contact, reason):
    install_fake_simulation(monkeypatch, getup_steps=3, getup_low=low,
                            getup_contact=contact, bad_pose_persists=True)
    result = evaluation.evaluate_case(config(go2_startup="upstream_getup", settle_s=2.))
    assert result["startup"]["completed"]
    assert result["startup"]["failure_reason"] is None
    assert result["failure_reason"] == reason
    assert result["progress"]["failure_control_phase"] == "settle"
    assert result["simulation_seconds"] == pytest.approx(.4)
    assert result["timing"]["evaluation_elapsed_s"] == pytest.approx(.1)
    assert result["timing"]["first_command_sim_time_s"] is None


@pytest.mark.parametrize("options,reason", [
    ({"getup_tilt": True}, "startup_tilt_limit"),
    ({"nonfinite": True}, "startup_nonfinite_state"),
    ({"never_ready": True}, "startup_timeout")])
def test_startup_failures_are_separate_and_json_serializable(monkeypatch, options, reason):
    install_fake_simulation(monkeypatch, getup_steps=30, **options)
    result = evaluation.evaluate_case(config(go2_startup="upstream_getup", startup_timeout_s=.3))
    assert result["failure_reason"] == reason
    assert result["startup"]["failure_reason"] == reason
    assert result["progress"]["failure_control_phase"] == "startup"
    assert result["timing"]["evaluation_clock_origin_sim_s"] is None
    assert not result["startup"]["completed"]
    assert result["post_startup_resources"] is None
    assert result["velocity_tracking"]["samples"] == 0
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("arguments", [
    ["--frictions", "0"], ["--frictions", "2.01"], ["--frictions", "nan"],
    ["--levels", "hard", "--difficulty", "2"]])
def test_invalid_geometry_arguments_fail_before_creating_output(tmp_path, arguments):
    output = tmp_path / "invalid"
    with pytest.raises(SystemExit) as caught:
        evaluation.main([*arguments, "--output", str(output)])
    assert caught.value.code == 2
    assert not output.exists()


def test_cli_crosses_go2_deployment_factors_and_friction_without_duplicate_go2w(tmp_path, monkeypatch):
    configs = []

    def worker(command, **kwargs):
        if command[0] == "git":
            return SimpleNamespace(stdout="fixture-base-commit\n", returncode=0)
        case = json.loads(command[command.index("--case-json") + 1])
        configs.append(case)
        Path(command[command.index("--case-output") + 1]).write_text(
            json.dumps({"case": case, "success": False, "failure_reason": "time_limit_before_finish"}))
        return SimpleNamespace(stdout="", returncode=0)

    monkeypatch.setattr(evaluation.subprocess, "run", worker)
    output = tmp_path / "factorial"
    assert evaluation.main(["--robots", "go2", "go2w", "--terrains", "ramp", "--levels", "hard",
                            "--speeds", ".3", "--go2-torque-modes", "configured", "upstream",
                            "--go2-startups", "crouched", "upstream_getup", "--frictions", ".2", ".8",
                            "--output", str(output)]) == 0
    go2 = [case for case in configs if case["robot"] == "go2"]
    go2w = [case for case in configs if case["robot"] == "go2w"]
    assert len(go2) == 8 and len(go2w) == 2
    assert len({(case["go2_torque_mode"], case["go2_startup"], case["terrain_friction"]) for case in go2}) == 8
    assert all(case["go2_torque_mode"] is None and case["go2_startup"] is None for case in go2w)
    assert {case["terrain_friction"] for case in go2w} == {.2, .8}
    assert all("-friction-" in path.name for path in (output / "cases").glob("*.json"))
    report = json.loads((output / "summary.json").read_text())
    assert report["completed_cases"] == report["planned_cases"] == 10
    assert report["source_provenance"]["worktree_base_commit"] == "fixture-base-commit"


def test_real_diffdrive_flat_route_ignores_supports_without_weakening_gates():
    result = evaluation.evaluate_case(config(robot="diffdrive", controller="auto", terrain="flat",
                                            terrain_level="standard", speed_m_s=.6, settle_s=.2))
    assert result["success"], result.get("failure_reason")
    assert result["torso_contact_steps"] == 0
    assert result["thresholds"]["max_tilt_deg"] == 50.
    assert result["thresholds"]["minimum_base_clearance_m"] == .1
