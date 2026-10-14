#!/usr/bin/env python3
"""Save an animated view of actual simulated robot motion (no ROS needed).

Run with the project's Python environment, for example:
  MUJOCO_GL=egl .venv/bin/python tools/render_preview.py
The GIF, final PNG and JSON metadata are new files; existing files are preserved.
"""
import argparse
from contextlib import closing
import json
import math
import os
from pathlib import Path


from project import ROOT


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=("go2", "diffdrive"), default="go2")
    parser.add_argument("--duration", type=float, default=5.)
    parser.add_argument("--settle", type=float, default=1.)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--cmd", type=float, nargs=3, default=(.3, 0., .15),
                        metavar=("VX", "VY", "WZ"))
    parser.add_argument("--output", type=Path, default=ROOT /
                        "output/validation-20261013/preview.gif")
    args = parser.parse_args(argv)
    if (not math.isfinite(args.duration) or args.duration <= 0
            or not math.isfinite(args.settle) or args.settle < 0
            or args.fps <= 0 or not all(map(math.isfinite, args.cmd))):
        parser.error("Require positive duration/fps and finite, nonnegative settle")
    if args.robot == "diffdrive" and args.cmd[1] != 0:
        parser.error("Differential drive does not support lateral velocity")
    if args.output.suffix.lower() != ".gif":
        parser.error("Output must have a .gif extension")
    artifacts = [args.output, args.output.with_suffix(".png"), args.output.with_suffix(".json")]
    for path in artifacts:
        if path.exists():
            raise FileExistsError(f"Refusing to replace existing preview: {path}")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    # The caller selects the backend before MuJoCo is imported; default is EGL.
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    from PIL import Image, ImageDraw, ImageFont
    from simple_rbt_mujoco.core import Simulation

    sim = Simulation(args.robot)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (.65, .1, .25)
    camera.distance = 3.1
    camera.azimuth = 125.
    camera.elevation = -23.
    options = mujoco.MjvOption()
    options.geomgroup[3:] = 0  # Hide collision proxies, preserving visible meshes.
    options.sitegroup[:] = 0  # Sensor/debug markers are not physical objects.
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 15)
    except OSError:
        font = ImageFont.load_default()
    images, samples = [], []
    frame_count = max(1, math.ceil(args.duration * args.fps))
    with closing(mujoco.Renderer(sim.model, height=400, width=640)) as renderer:
        for frame_index in range(1, frame_count + 1):
            target = min(frame_index / args.fps, args.duration)
            while sim.data.time + 1e-12 < target:
                sim.set_command(*(args.cmd if sim.data.time + 1e-12 >= args.settle
                                  else (0., 0., 0.)))
                sim.step()
            renderer.update_scene(sim.data, camera=camera, scene_option=options)
            image = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 640, 51), fill=(22, 28, 35))
            draw.text((12, 7), f"MuJoCo | {args.robot} | simulation {sim.data.time:.2f} s",
                      fill=(245, 248, 250), font=font)
            draw.text((12, 28), f"command: forward {sim.command[0]:.2f} m/s, turn {sim.command[2]:.2f} rad/s",
                      fill=(190, 210, 225), font=font)
            images.append(image)
            position, quaternion = sim.pose()
            samples.append({"stamp_ns": sim.stamp_ns, "position_m": position.tolist(),
                            "quaternion_xyzw": quaternion.tolist(),
                            "command_m_s_rad_s": sim.command.tolist()})
    images[0].save(args.output, save_all=True, append_images=images[1:],
                   duration=round(1000 / args.fps), loop=0, disposal=2)
    images[-1].save(args.output.with_suffix(".png"))
    metadata = {
        "source": "Actual MuJoCo Simulation physics; fixed third-person camera",
        "robot": args.robot, "mujoco_version": mujoco.__version__,
        "requested_duration_s": args.duration, "simulation_duration_s": sim.data.time,
        "settle_s": args.settle, "requested_command_m_s_rad_s": list(args.cmd),
        "frame_count": len(images), "requested_fps": args.fps,
        "gif_frame_duration_ms": round(1000 / args.fps),
        "image_size": [640, 400],
        "camera": {"lookat": camera.lookat.tolist(), "distance": camera.distance,
                   "azimuth_deg": camera.azimuth, "elevation_deg": camera.elevation},
        "samples": samples,
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps({"gif": str(args.output), "frames": len(images),
                      "simulation_seconds": sim.data.time,
                      "final_position_m": samples[-1]["position_m"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
