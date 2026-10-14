#!/usr/bin/env python3
"""Render real full-course robot traversals into individual and tiled GIFs.

Each panel follows a real simulated attempt through finish or a stated failure.
The start-up settle is omitted from playback, but simulation timestamps are
shown. Completed and failed panels both hold their labelled final frame.
"""
from __future__ import annotations

import argparse
from contextlib import closing
import json
import math
import os
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


def gif_durations_ms(frame_count, playback_fps, final_hold_ms=1500):
    """Round cumulative GIF timestamps, avoiding per-frame timing drift."""
    boundaries = [round(index * 100 / playback_fps) for index in range(frame_count + 1)]
    durations = [10 * (end - start) for start, end in zip(boundaries, boundaries[1:])]
    durations[-1] += final_hold_ms
    return durations


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", nargs="+", choices=("go2", "go2w"), default=["go2", "go2w"])
    parser.add_argument("--terrains", nargs="+", choices=("flat", "ramp", "stairs", "rough"), default=["ramp", "stairs", "rough"])
    parser.add_argument("--go2-controller", choices=("himloco", "legacy", "robot_lab"), default="robot_lab")
    parser.add_argument("--go2-torque-mode", choices=("configured", "upstream"), default="configured")
    parser.add_argument("--go2-startup", choices=("crouched", "upstream_getup"), default="crouched")
    parser.add_argument("--control-mode", choices=("native", "tracked"), default="tracked")
    parser.add_argument("--terrain-level", choices=("standard", "moderate", "hard", "extreme"), default="standard")
    parser.add_argument("--speed", type=float, default=.6)
    parser.add_argument("--difficulty", type=float, default=1.)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--friction", type=float)
    parser.add_argument("--settle", type=float, default=2.)
    parser.add_argument("--capture-fps", type=int, default=8)
    parser.add_argument("--playback-fps", type=int, default=12)
    parser.add_argument("--output", type=Path, required=True, help="New artifact directory")
    args = parser.parse_args(argv)
    if (not math.isfinite(args.speed) or not 0 < args.speed <= 1
            or not math.isfinite(args.difficulty) or not 0 < args.difficulty <= 3
            or not math.isfinite(args.settle) or args.settle < 0
            or args.capture_fps <= 0 or not 0 < args.playback_fps <= 100 or args.seed < 0):
        parser.error("Invalid speed/difficulty/settle/FPS/seed")
    if args.terrain_level != "standard" and args.difficulty != 1.:
        parser.error("Named terrain levels require --difficulty 1")
    if args.friction is not None and (not math.isfinite(args.friction) or not 0 < args.friction <= 2):
        parser.error("Friction must be finite and in (0,2]")
    if args.go2_controller == "legacy" and (args.go2_torque_mode != "configured" or args.go2_startup != "crouched"):
        parser.error("Go2 deployment alternatives require himloco or robot_lab")
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    from simple_rbt_mujoco.core import Simulation

    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 13)
        title_font = ImageFont.truetype("DejaVuSans.ttf", 17)
    except OSError:
        font = title_font = ImageFont.load_default()
    robots, terrains = list(dict.fromkeys(args.robots)), list(dict.fromkeys(args.terrains))
    width, height = 480, 300
    panels, cases = [], []
    final_hold_ms = 1500
    failure_labels = {"tilt_limit": "STOPPED: tilt exceeded 50 degrees",
                      "base_clearance_limit": "STOPPED: base clearance below 0.10 m",
                      "torso_environment_contact": "STOPPED: torso touched the environment",
                      "left_course_corridor": "STOPPED: left the course corridor",
                      "time_limit_before_finish": "STOPPED: time limit before finish"}
    for robot in robots:
        for terrain in terrains:
            controller = args.go2_controller if robot == "go2" else "rl_sar"
            deployment = {"go2_torque_mode": args.go2_torque_mode, "go2_startup": args.go2_startup} if robot == "go2" else {}
            sim = Simulation(robot, terrain=terrain, difficulty=args.difficulty,
                             seed=args.seed, controller=controller,
                             velocity_feedback=args.control_mode == "tracked",
                             terrain_level=args.terrain_level, terrain_friction=args.friction,
                             **deployment)
            while not sim.startup_complete:
                if sim.data.time >= 10.:
                    raise RuntimeError("Go2 startup watchdog expired before preview")
                sim.set_command(0., 0., 0.)
                sim.step()
            policy_origin = float(sim.data.time)
            profile = sim.terrain
            geometry_label = terrain
            if terrain == "ramp":
                geometry_label += f" {profile.parameters['slope_degrees']:g}deg"
            elif terrain == "stairs":
                geometry_label += f" {100 * profile.parameters['step_height_m']:g}cm x{profile.parameters['ascending_steps']}"
            camera = mujoco.MjvCamera()
            camera.distance = max(2.3, 2.2 + profile.metadata["max_height_m"])
            # A side view keeps the camera's X at lookat X, inside the east
            # wall even at the longer 10.6 m finish. A trailing oblique view
            # can cross that wall and hide the final robot behind the wall.
            camera.azimuth, camera.elevation = 90., -23.
            options = mujoco.MjvOption()
            options.geomgroup[3:] = 0
            options.sitegroup[:] = 0
            frames, samples = [], []
            next_capture = policy_origin + args.settle
            distance_budget = max(8., profile.finish_x + 2.)
            max_time = policy_origin + args.settle + distance_budget / args.speed
            base_id = int(sim.model.site_bodyid[sim.model.site("base_origin").id])
            entered_course = False
            failure = None
            success = False
            with closing(mujoco.Renderer(sim.model, height=height, width=width)) as renderer:
                while sim.data.time < max_time:
                    sim.set_command(args.speed if sim.data.time >= policy_origin + args.settle else 0., 0., 0.)
                    sim.step()
                    position, quaternion = sim.pose()
                    rotation = sim.data.site_xmat[sim.model.site("base_origin").id].reshape(3, 3)
                    tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1., 1.))))
                    terrain_height = profile.height_at(position[0], position[1])
                    clearance = float(position[2]) - terrain_height
                    entered_course = entered_course or position[0] >= profile.entry_x
                    torso_contacts = []
                    for contact in sim.data.contact:
                        a, b = int(contact.geom1), int(contact.geom2)
                        body_a, body_b = int(sim.model.geom_bodyid[a]), int(sim.model.geom_bodyid[b])
                        if {body_a, body_b} == {0, base_id}:
                            torso_contacts.append(sim.model.geom(a if body_a == base_id else b).name or "unnamed")
                    if tilt > 50.:
                        failure = "tilt_limit"
                    elif clearance < .1:
                        failure = "base_clearance_limit"
                    elif torso_contacts:
                        failure = "torso_environment_contact"
                    elif entered_course and abs(position[1]) > profile.width / 2. - .25:
                        failure = "left_course_corridor"
                    elif position[0] >= profile.finish_x:
                        success = True
                    elif sim.data.time + 1e-12 >= max_time:
                        failure = "time_limit_before_finish"
                    finished = success or failure is not None
                    if sim.data.time + 1e-12 >= next_capture or finished:
                        camera.lookat[:] = (position[0] + .3, position[1], max(.18, position[2] - .05))
                        renderer.update_scene(sim.data, camera=camera, scene_option=options)
                        frame = Image.fromarray(renderer.render())
                        draw = ImageDraw.Draw(frame)
                        draw.rectangle((0, 0, width, 43), fill=(22, 28, 35))
                        draw.text((9, 5), f"{robot.upper()} | {geometry_label} ({args.terrain_level}) | {sim.controller_name}", font=font, fill=(245, 248, 250))
                        velocity = sim.velocity()[0]
                        draw.text((9, 24), f"sim {sim.data.time:.2f}s | X {position[0]:.2f}/{profile.finish_x:g}m | vx {velocity[0]:.2f}m/s", font=font, fill=(185, 210, 225))
                        draw.rectangle((0, height - 6, width, height), fill=(30, 40, 46))
                        progress = float(np.clip(position[0] / profile.finish_x, 0., 1.))
                        draw.rectangle((0, height - 6, round(width * progress), height), fill=(79, 199, 162))
                        if success or failure:
                            draw.rectangle((0, height - 49, width, height - 6), fill=(22, 28, 35))
                            label = f"FINISHED {profile.finish_x:g}m" if success else failure_labels[failure]
                            draw.text((9, height - 46), label,
                                      font=font, fill=(230, 255, 240) if success else (255, 160, 150))
                            draw.text((9, height - 27), f"tilt {tilt:.1f}deg | base clearance {clearance:.2f}m",
                                      font=font, fill=(185, 210, 225))
                        frames.append(frame)
                        samples.append({"stamp_ns": sim.stamp_ns, "position_m": position.tolist(),
                                        "quaternion_xyzw": quaternion.tolist(),
                                        "velocity_base_m_s": velocity.tolist(), "tilt_deg": tilt,
                                        "terrain_height_m": terrain_height, "base_clearance_m": clearance,
                                        "torso_contact_geoms": sorted(set(torso_contacts)),
                                        "success": success, "failure_reason": failure})
                        next_capture += 1. / args.capture_fps
                    if finished:
                        break
            if not success and failure is None:
                failure = "time_limit_before_finish"
            stem = f"{robot}-{terrain}-{args.terrain_level}-{args.control_mode}"
            durations = gif_durations_ms(len(frames), args.playback_fps, final_hold_ms)
            frames[0].save(args.output / (stem + ".gif"), save_all=True,
                           append_images=frames[1:], duration=durations,
                           loop=0, disposal=2)
            midpoint = min(range(len(samples)), key=lambda i: abs(samples[i]["position_m"][0] - profile.finish_x / 2.))
            frames[midpoint].save(args.output / (stem + ".png"))
            frames[-1].save(args.output / (stem + "-final.png"))
            panels.append(frames)
            cases.append({"robot": robot, "controller": sim.controller_name,
                          "deployment": sim.deployment_metadata,
                          "policy_clock_origin_s": policy_origin,
                          "control_mode": args.control_mode, "terrain": sim.terrain.metadata,
                          "command_m_s_rad_s": [args.speed, 0., 0.], "success": success,
                          "failure_reason": failure, "frames": len(frames),
                          "simulation_end_s": sim.data.time,
                          "maximum_simulation_duration_s": max_time,
                          "commanded_distance_budget_m": distance_budget,
                          "camera": {"distance_m": camera.distance,
                                     "azimuth_deg": camera.azimuth,
                                     "elevation_deg": camera.elevation},
                          "final_png": stem + "-final.png",
                          "gif_frame_durations_ms": durations,
                          "gif_total_duration_ms": sum(durations),
                          "gif": stem + ".gif", "samples": samples})
            print(f"{stem}: {'finished' if success else failure}, {len(frames)} frames", flush=True)

    banner = 62
    mosaic_frames = []
    total_frames = max(len(panel) for panel in panels)
    for index in range(total_frames):
        mosaic = Image.new("RGB", (width * len(terrains), height * len(robots) + banner), (19, 25, 32))
        draw = ImageDraw.Draw(mosaic)
        draw.text((12, 9), f"Real MuJoCo attempts | {args.terrain_level} | command {args.speed:g} m/s", font=title_font, fill=(245, 248, 250))
        feedback = "ideal simulated velocity/heading feedback" if args.control_mode == "tracked" else "native policy commands"
        draw.text((12, 35), f"{args.control_mode}: {feedback} | replay {args.playback_fps / args.capture_fps:g}x | ended panels hold", font=font, fill=(170, 195, 211))
        for panel_index, panel in enumerate(panels):
            row, col = divmod(panel_index, len(terrains))
            mosaic.paste(panel[min(index, len(panel) - 1)], (col * width, banner + row * height))
        mosaic_frames.append(mosaic)
    mosaic_durations = gif_durations_ms(len(mosaic_frames), args.playback_fps, final_hold_ms)
    mosaic_frames[0].save(args.output / "terrain-preview.gif", save_all=True,
                          append_images=mosaic_frames[1:], duration=mosaic_durations,
                          loop=0, disposal=2)
    mosaic_frames[len(mosaic_frames) // 2].save(args.output / "terrain-preview.png")
    mosaic_frames[-1].save(args.output / "terrain-preview-final.png")
    manifest = {"source": "Actual MuJoCo physics and mesh rendering; no illustration",
                "mujoco_version": mujoco.__version__, "capture_fps": args.capture_fps,
                "playback_fps": args.playback_fps,
                "gif_frame_durations_ms": mosaic_durations,
                "gif_total_duration_ms": sum(mosaic_durations),
                "gif_timing": "Cumulative timestamps rounded to nearest 10 ms, then explicit final hold added",
                "final_result_hold_ms": final_hold_ms,
                "settle_omitted_s": args.settle, "gif": "terrain-preview.gif",
                "terrain_level": args.terrain_level,
                "failure_interpretation": "Stopped at the first evaluation limit; base clearance is vertical distance from base origin to the surface directly underneath, not proof of a fallen robot. Torso contact is measured separately.",
                "png_selection": "individual panels nearest half of actual finish X; mosaic midway through playback; -final PNGs show every result",
                "command": [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]], "cases": cases}
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"preview": str(args.output / "terrain-preview.gif"),
                      "cases": len(cases), "finished": sum(case["success"] for case in cases)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
