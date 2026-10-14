#!/usr/bin/env python3
"""Compare native physical trajectories to the retained Python implementation.

Offline test only: the C++ executable never launches this script. Every tick is
compared, including sensor values after mj_forward and the initial t=0 snapshot.
This short transition suite verifies deployment equivalence, not route success.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from project import ROOT
import mujoco
import numpy as np
from simple_rbt_mujoco.core import Simulation
from simple_rbt_mujoco.robots import NO_LATERAL_ROBOTS, ROBOT_NAMES

# Fixed diagnostic tolerances. Failures are reported, never auto-relaxed.
ATOL = {"qpos": 1e-8, "qvel": 1e-7, "ctrl": 1e-6, "sensordata": 1e-5}
MODEL_FIELDS = (
    "qpos0", "body_pos", "body_quat", "body_mass", "body_inertia", "body_ipos", "body_iquat",
    "jnt_type", "jnt_pos", "jnt_axis", "jnt_range", "dof_damping", "dof_armature", "dof_frictionloss",
    "geom_type", "geom_bodyid", "geom_pos", "geom_quat", "geom_size", "geom_friction", "geom_priority",
    "geom_condim", "geom_contype", "geom_conaffinity", "geom_solref", "geom_solimp",
    "actuator_trnid", "actuator_gainprm", "actuator_biasprm", "actuator_dynprm", "actuator_gear",
    "actuator_ctrlrange", "actuator_forcerange", "actuator_ctrllimited", "actuator_forcelimited",
    "sensor_type", "sensor_objid", "sensor_adr", "sensor_dim", "site_pos", "site_quat", "site_bodyid",
    "key_qpos", "key_ctrl",
)


def cases(args):
    for robot in args.robots:
        controllers = args.go2_controllers if robot == "go2" else ["auto"]
        for controller in controllers:
            alternatives = [("configured", "crouched")]
            if robot == "go2" and controller != "legacy" and args.suite == "all":
                alternatives += [("upstream", "crouched"), ("configured", "upstream_getup"),
                                 ("upstream", "upstream_getup")]
            for mode in args.control_modes:
                # This reference chassis has no outer tracker. Both modes
                # produced identical models, full trajectories and metadata;
                # keep explicit tracked-only runs, omit the duplicate matrix.
                if robot == "diffdrive" and mode == "tracked" and "native" in args.control_modes:
                    continue
                for torque, startup in alternatives:
                    options = dict(project_root=str(ROOT), robot=robot, controller=controller,
                                   control_mode=mode, go2_torque_mode=torque, go2_startup=startup,
                                   terrain=args.terrain, terrain_level=args.terrain_level,
                                   difficulty=args.difficulty, seed=args.seed, friction=args.friction)
                    label = f"{robot}-{controller}-{mode}-{torque}-{startup}"
                    if args.case_filter and args.case_filter not in label:
                        continue
                    segments = [dict(ticks=40, command=[0., 0., 0.]),
                                dict(ticks=41, command=[.3, 0., 0.]),
                                dict(ticks=43, command=[.3, 0., .2]),
                                dict(ticks=47, command=[-.2, 0., -.15]),
                                dict(ticks=53, command=[0., 0., 0.])]
                    # Sampling crosses both 20 ms policy and nonintegral 5 ms PD deadlines.
                    if robot not in NO_LATERAL_ROBOTS:
                        segments.insert(4, dict(ticks=49, command=[0., .2, .3]))
                    if startup == "upstream_getup":
                        # Nonzero command throughout GetUp must not reach policy/tracker.
                        segments.insert(0, dict(ticks=640, command=[.3, 0., 0.]))
                    rejected = [[0., .1, 0.]] if robot in NO_LATERAL_ROBOTS else []
                    if robot == "fr_mini":
                        rejected += [[0., 0., .2], [1e-10, 0., -.2]]
                    yield label, dict(options=options, segments=segments, reject_commands=rejected)


def model_comparison(expected, actual):
    errors = {}
    for name in MODEL_FIELDS:
        a, b = np.asarray(getattr(expected, name)), np.asarray(getattr(actual, name))
        if not np.array_equal(a, b):
            errors[name] = dict(expected_shape=a.shape, actual_shape=b.shape)
            if a.shape == b.shape:
                errors[name].update(different=int(np.sum(a != b)), max_abs=float(np.max(np.abs(a-b))))
    for name in ("timestep", "integrator", "cone", "iterations", "solver", "tolerance", "gravity"):
        a, b = np.asarray(getattr(expected.opt, name)), np.asarray(getattr(actual.opt, name))
        if not np.array_equal(a, b):
            errors["opt."+name] = dict(expected=a.tolist(), actual=b.tolist())
    return errors


def compare_array(reference, actual, tolerance):
    if reference.shape != actual.shape:
        return dict(passed=False, expected_shape=reference.shape, actual_shape=actual.shape)
    error = np.abs(reference - actual)
    mismatch = ~np.isfinite(actual) | (error > tolerance)
    indices = np.argwhere(mismatch)
    return dict(passed=not len(indices), tolerance=tolerance, rtol=0.,
                max_abs=float(error.max(initial=0.)), different=int(np.sum(reference != actual)),
                first_failure=indices[0].tolist() if len(indices) else None,
                first_failure_expected=float(reference[tuple(indices[0])]) if len(indices) else None,
                first_failure_actual=float(actual[tuple(indices[0])]) if len(indices) else None)


def run_case(label, test, binary, output):
    folder = output / label
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "case.json").write_text(json.dumps(test, indent=2)+"\n")
    started = time.perf_counter()
    run = subprocess.run([str(binary), str(folder/"case.json"), str(folder)],
                         text=True, capture_output=True, timeout=120,
                         env={**os.environ, "OPENBLAS_NUM_THREADS":"1", "OMP_NUM_THREADS":"1"})
    report = dict(case=label, native_returncode=run.returncode, native_stderr=run.stderr,
                  native_stdout=run.stdout, passed=False)
    if run.returncode:
        (folder / "comparison.json").write_text(json.dumps(report, indent=2)+"\n")
        return report
    options = test["options"]
    sim = Simulation(options["robot"], controller=options["controller"],
                     velocity_feedback=options["control_mode"] == "tracked",
                     terrain=options["terrain"], terrain_level=options["terrain_level"],
                     difficulty=options["difficulty"], seed=options["seed"],
                     terrain_friction=None if options["friction"] == -1 else options["friction"],
                     go2_torque_mode=options["go2_torque_mode"], go2_startup=options["go2_startup"])
    native_model = mujoco.MjModel.from_binary_path(str(folder/"native-model.mjb"))
    report["model_differences"] = model_comparison(sim.model, native_model)
    for command in test["reject_commands"]:
        sim.set_command(.2, 0., 0.)
        try:
            sim.set_command(*command)
        except ValueError:
            assert np.array_equal(sim.command, np.zeros(3))
        else:
            raise AssertionError("Python oracle accepted unsupported command")
    sim.set_command(0., 0., 0.)
    rows, stamps, startup, commands = [], [], [], []
    def capture():
        rows.append(np.concatenate([sim.data.qpos, sim.data.qvel, sim.data.ctrl, sim.data.sensordata]))
        stamps.append(sim.stamp_ns)
        startup.append([sim.startup_complete, sim.startup_phase, sim.startup_steps])
        commands.append(sim.command.tolist())
    capture()
    for segment in test["segments"]:
        sim.set_command(*segment["command"])
        for _ in range(segment["ticks"]):
            sim.step()
            capture()
    reference = np.asarray(rows, dtype=np.float64)
    with (folder / "native-trajectory.f64").open("rb") as stream:
        header = np.fromfile(stream, np.uint64, 5)
        actual = np.fromfile(stream, np.float64).reshape(int(header[0]), int(np.sum(header[1:])))
    with (folder / "python-trajectory.f64").open("wb") as stream:
        header.tofile(stream)
        reference.tofile(stream)
    meta = json.loads((folder / "native-metadata.json").read_text())
    report["rows"] = len(reference)
    report["arrays"] = {}
    offset = 0
    for key, size in zip(ATOL, header[1:]):
        size = int(size)
        report["arrays"][key] = compare_array(reference[:,offset:offset+size], actual[:,offset:offset+size], ATOL[key])
        offset += size
    report["stamps_equal"] = stamps == meta["stamp_ns"]
    report["startup_equal"] = startup == meta["startup"]
    report["commands_equal"] = commands == meta["command"]
    report["controller_equal"] = sim.controller_name == meta["controller"]
    report["command_limits_equal"] = sim.command_limits.tolist() == meta["command_limits"]
    report["counters"] = {}
    if sim.controller is not None:
        for key in ("policy_calls", "pd_calls"):
            if hasattr(sim.controller, key):
                expected = getattr(sim.controller, key)
                observed = meta["deployment"][key]
                report["counters"][key] = dict(expected=expected, actual=observed, passed=expected==observed)
    if sim.robot in ("go2", "go2w"):
        expected_calls = (sim.controller.steps + sim.controller.substeps - 1) // sim.controller.substeps
        report["counters"]["policy_calls"] = dict(expected=expected_calls,
              actual=meta["deployment"]["policy_calls"], passed=expected_calls==meta["deployment"]["policy_calls"])
    report["passed"] = (not report["model_differences"] and all(r["passed"] for r in report["arrays"].values())
                        and report["stamps_equal"] and report["startup_equal"] and report["commands_equal"]
                        and report["controller_equal"] and report["command_limits_equal"] and all(r["passed"] for r in report["counters"].values()))
    report["elapsed_wall_s_not_performance_benchmark"] = time.perf_counter()-started
    (folder / "python-metadata.json").write_text(json.dumps(dict(stamp_ns=stamps, startup=startup,
                    command=commands, controller=sim.controller_name, deployment=sim.deployment_metadata), indent=2)+"\n")
    (folder / "comparison.json").write_text(json.dumps(report, indent=2)+"\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New output directory; existing cases are never overwritten")
    parser.add_argument("--robots", nargs="+", choices=ROBOT_NAMES, default=list(ROBOT_NAMES))
    parser.add_argument("--go2-controllers", nargs="+", choices=("robot_lab","himloco","legacy"), default=["robot_lab","himloco","legacy"])
    parser.add_argument("--control-modes", nargs="+", choices=("native","tracked"), default=["native","tracked"])
    parser.add_argument("--suite", choices=("basic","all"), default="all")
    parser.add_argument("--terrain", default="flat")
    parser.add_argument("--terrain-level", default="standard")
    parser.add_argument("--difficulty", type=float, default=1.)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--friction", type=float, default=-1.)
    parser.add_argument("--case-filter")
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for label, test in cases(args):
        report = run_case(label, test, args.binary.resolve(), args.output)
        results.append(report)
        failed = [name for name, value in report.get("arrays", {}).items() if not value["passed"]]
        print(json.dumps(dict(case=label, passed=report["passed"], failed_arrays=failed,
                              model_differences=list(report.get("model_differences", {})))), flush=True)
    if not results:
        parser.error("No cases selected")
    sources = [ROOT/"src/core.cpp", ROOT/"tests/core_oracle.cpp", Path(__file__),
               ROOT/"assets/catalog.json", ROOT/"reference/python/simple_rbt_mujoco/core.py"]
    summary = dict(schema=1, passed=sum(r["passed"] for r in results), cases=len(results),
                   tolerances=ATOL, rtol=0., argv=sys.argv, results=results,
                   sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (args.output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(f"{summary['passed']}/{summary['cases']} cases passed; {args.output/'summary.json'}")
    return 0 if summary["passed"] == summary["cases"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
