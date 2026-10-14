"""Deterministic physical terrain courses built from raycastable MuJoCo boxes.

The caller supplies a floor at world Z=0. Courses run along +X, beginning at
X=1 m; completion is at least X=6 m and beyond the final feature. Heights describe
the actual collision solids, not a separate visual approximation. All geometry
uses ordinary boxes supported by MuJoCo CPU and MuJoCo-LiDAR's Warp backend.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np


TERRAIN_NAMES = ("flat", "ramp", "stairs", "rough", "hills")
TERRAIN_LEVELS = ("standard", "moderate", "hard", "extreme")
_RAMP_LEVELS = {"moderate": (15., 3., 1.), "hard": (20., 4., 1.),
                "extreme": (25., 4., 1.)}  # degrees, horizontal run, landing
_STAIR_LEVELS = {"moderate": (6, .30, .12, .9), "hard": (6, .30, .18, .9),
                 "extreme": (8, .30, .22, 1.)}  # count, tread, rise, landing
_ROUGH_LEVELS = {"moderate": .05, "hard": .08, "extreme": .12}
_FRICTION = (1.0, 0.01, 0.001)


@dataclass(frozen=True)
class _Box:
    name: str
    centre: tuple[float, float, float]
    half_size: tuple[float, float, float]
    slope: float = 0.0  # radians: positive slopes rise along world +X

    def upper_surface(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Upper intersection of a vertical line with this exact oriented box."""
        dx = x - self.centre[0]
        c, s = math.cos(self.slope), math.sin(self.slope)
        hx, hy, hz = self.half_size
        # world->local: local_x=c*dx+s*dz, local_z=-s*dx+c*dz.
        low = (-hz + s * dx) / c
        high = (hz + s * dx) / c
        inside = np.abs(y - self.centre[1]) <= hy
        if abs(s) < 1e-14:
            inside = inside & (np.abs(dx) <= hx)
        else:
            bounds_a, bounds_b = (-hx - c * dx) / s, (hx - c * dx) / s
            low = np.maximum(low, np.minimum(bounds_a, bounds_b))
            high = np.minimum(high, np.maximum(bounds_a, bounds_b))
            inside = inside & (low <= high)
        return np.where(inside, self.centre[2] + high, -np.inf)

    def append_xml(self, world: ET.Element, colour: tuple[float, ...]) -> None:
        def values(items):
            return " ".join(format(float(value), ".17g") for value in items)

        ET.SubElement(world, "geom", name=self.name, type="box",
                      pos=values(self.centre), size=values(self.half_size),
                      quat=values((math.cos(self.slope / 2.), 0.,
                                   -math.sin(self.slope / 2.), 0.)),
                      rgba=values(colour), friction=values(_FRICTION),
                      group="1", contype="1", conaffinity="1", condim="3")


@dataclass(frozen=True)
class TerrainProfile:
    name: str
    level: str
    difficulty: float
    seed: int
    entry_x: float
    exit_x: float
    finish_x: float
    width: float
    parameters: dict[str, Any]
    _boxes: tuple[_Box, ...]
    friction: tuple[float, float, float] = _FRICTION

    def height_at(self, x, y=0.0):
        """Highest collision surface at world (x,y), including the Z=0 floor.

        Scalar inputs return a float; broadcastable array inputs return an
        array. At a vertical step edge the higher adjacent surface is returned.
        This is terrain clearance, not an estimate of a robot's support polygon.
        """
        x, y = np.broadcast_arrays(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
        if not np.isfinite(x).all() or not np.isfinite(y).all():
            raise ValueError("Terrain height coordinates must be finite")
        height = np.zeros_like(x)
        for box in self._boxes:
            height = np.maximum(height, box.upper_surface(x, y))
        return float(height) if height.ndim == 0 else height

    @property
    def metadata(self) -> dict[str, Any]:
        """JSON-ready course definition suitable for benchmark records."""
        highest = max((box.centre[2] + abs(math.sin(box.slope)) * box.half_size[0]
                       + math.cos(box.slope) * box.half_size[2]
                       for box in self._boxes), default=0.)
        return {"name": self.name, "level": self.level,
                "difficulty": self.difficulty, "seed": self.seed,
                "entry_x_m": self.entry_x, "exit_x_m": self.exit_x,
                "finish_x_m": self.finish_x, "width_m": self.width,
                "max_height_m": highest, "geom_count": len(self._boxes),
                "parameters": dict(self.parameters), "friction": list(self.friction),
                "height_reference": "world Z=0 floor; exact upper box surface",
                "completion": "base X >= finish_x_m while upright and inside course width"}


def _level_box(name, begin, end, width, height):
    # Extend below the floor to eliminate thin gaps at the ground boundary.
    bottom = -.10
    return _Box(name, ((begin + end) / 2., 0., (height + bottom) / 2.),
                ((end - begin) / 2., width / 2., (height - bottom) / 2.))


def _sloped_box(name, begin, end, width, initial_height, angle):
    # Position the TOP face exactly at (begin, initial_height). Its horizontal
    # span is end-begin. Thickness extends beneath that face, never above it.
    thickness = .15
    c, s = math.cos(angle), math.sin(angle)
    rise = (end - begin) * math.tan(angle)
    return _Box(name,
                ((begin + end) / 2. + s * thickness / 2., 0.,
                 initial_height + rise / 2. - c * thickness / 2.),
                ((end - begin) / (2. * c), width / 2., thickness / 2.), angle)


def append_terrain(root: ET.Element, name: str = "flat", *,
                   level: str = "standard", difficulty: float = 1.0,
                   seed: int = 0) -> TerrainProfile:
    """Append one named physical course to an MJCF root and return its profile.

    ``standard`` preserves the original 8-degree ramp, four 6 cm stairs with
    40 cm treads, and 0–2.5 cm rough cells. Its ``difficulty`` in (0,3] scales
    slope angle or vertical feature height without changing route dimensions.
    ``moderate``, ``hard`` and ``extreme`` specify larger fixed courses and
    require difficulty=1; every ramp and stair course includes a descent.
    Rough terrain is deterministic for the same seed. ``hills`` is a continuous
    two-wave profile, built from 48 joined sloped boxes over 4 m. Peak heights
    follow the rough levels but contain no vertical steps; slopes are piecewise
    constant rather than a continuously differentiable analytic surface.
    No robot, existing
    floor, lights or arena obstacles are modified; the caller must keep the
    course unobstructed and provide room through profile.finish_x.
    """
    if name not in TERRAIN_NAMES:
        raise ValueError(f"Unknown terrain {name!r}; choose one of {TERRAIN_NAMES}")
    if level not in TERRAIN_LEVELS:
        raise ValueError(f"Unknown terrain level {level!r}; choose one of {TERRAIN_LEVELS}")
    difficulty = float(difficulty)
    if not math.isfinite(difficulty) or not 0. < difficulty <= 3.:
        raise ValueError("Terrain difficulty must be finite and in (0, 3]")
    if level != "standard" and difficulty != 1.:
        raise ValueError("Named terrain levels other than standard require difficulty=1")
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or seed < 0:
        raise ValueError("Terrain seed must be a nonnegative integer")
    seed = int(seed)
    world = root.find("worldbody")
    if root.tag != "mujoco" or world is None:
        raise ValueError("Terrain requires an MJCF root containing worldbody")
    if any(geom.get("name", "").startswith("terrain_") for geom in world.iter("geom")):
        raise ValueError("A terrain course has already been added to this scene")

    start, width, finish = 1.0, 2.4, 6.0
    boxes: list[_Box] = []
    parameters: dict[str, Any] = {}
    end = start
    if name == "ramp":
        degrees, run, landing = ((8. * difficulty, 2., .6) if level == "standard"
                                 else _RAMP_LEVELS[level])
        angle = math.radians(degrees)
        peak = run * math.tan(angle)
        top_begin, top_end = start + run, start + run + landing
        end = top_end + run
        boxes = [
            _sloped_box("terrain_ramp_up", start, top_begin, width, 0., angle),
            _level_box("terrain_ramp_landing", top_begin, top_end, width, peak),
            _sloped_box("terrain_ramp_down", top_end, end, width, peak, -angle),
        ]
        parameters = {"slope_degrees": degrees, "horizontal_run_m": run,
                      "landing_length_m": landing, "slab_thickness_m": .15}
    elif name == "stairs":
        count, tread, step, landing = ((4, .4, .06 * difficulty, .8) if level == "standard"
                                       else _STAIR_LEVELS[level])
        for i in range(count):
            boxes.append(_level_box(f"terrain_stair_up_{i}", start + i * tread,
                                    start + (i + 1) * tread, width, (i + 1) * step))
        top_begin, top_end = start + count * tread, start + count * tread + landing
        boxes.append(_level_box("terrain_stair_landing", top_begin, top_end,
                                width, count * step))
        for i in range(count - 1):
            boxes.append(_level_box(f"terrain_stair_down_{i}", top_end + i * tread,
                                    top_end + (i + 1) * tread, width, (count - i - 1) * step))
        end = top_end + (count - 1) * tread
        parameters = {"ascending_steps": count, "step_height_m": step,
                      "tread_length_m": tread, "landing_length_m": landing,
                      "descending_risers": count}
    elif name == "rough":
        cell, columns, rows = .4, 10, 6
        max_height = .025 * difficulty if level == "standard" else _ROUGH_LEVELS[level]
        overlap = .00002  # 20 micrometres avoids CPU/float32 cracks at shared edges.
        generator = np.random.default_rng(seed)
        heights = generator.uniform(0., max_height, size=(columns, rows))
        for column in range(columns):
            for row in range(rows):
                height = float(heights[column, row])
                bottom = -.10
                x0 = max(start, start + column * cell - overlap / 2.)
                x1 = min(start + columns * cell, start + (column + 1) * cell + overlap / 2.)
                y0 = max(-width / 2., -width / 2. + row * cell - overlap / 2.)
                y1 = min(width / 2., -width / 2. + (row + 1) * cell + overlap / 2.)
                boxes.append(_Box(f"terrain_rough_{column}_{row}",
                                  ((x0 + x1) / 2., (y0 + y1) / 2.,
                                   (height + bottom) / 2.),
                                  ((x1 - x0) / 2., (y1 - y0) / 2., (height - bottom) / 2.)))
        end = start + columns * cell
        parameters = {"cell_size_m": cell, "columns": columns, "rows": rows,
                      "maximum_cell_height_m": max_height, "internal_cell_overlap_m": overlap,
                      "cell_heights_m": heights.tolist()}
    elif name == "hills":
        # Two continuous waves sampled into 48 joined sloped solids. Top faces
        # meet at exactly the same height; there are slope kinks but no risers.
        # Keeping ordinary boxes preserves CPU/Warp ray and collision parity.
        length, segments, waves = 4., 48, 2
        amplitude = .025 * difficulty if level == "standard" else _ROUGH_LEVELS[level]
        xs = np.linspace(start, start + length, segments + 1)
        heights = amplitude * np.sin(np.linspace(0., waves * math.pi, segments + 1)) ** 2
        heights[0] = heights[-1] = 0.
        for i in range(segments):
            slope = math.atan2(float(heights[i + 1] - heights[i]), float(xs[i + 1] - xs[i]))
            boxes.append(_sloped_box(f"terrain_hills_{i}", float(xs[i]), float(xs[i + 1]),
                                     width, float(heights[i]), slope))
        end = start + length
        parameters = {"amplitude_m": amplitude, "wavelength_m": length / waves,
                      "wave_count": waves, "segment_count": segments,
                      "node_x_m": xs.tolist(), "node_height_m": heights.tolist(),
                      "surface": "continuous piecewise-planar longitudinal waves; no vertical risers",
                      "height_is_continuous": True, "slope_is_continuous": False,
                      "maximum_segment_slope_degrees": math.degrees(max(abs(box.slope) for box in boxes))}
    else:
        parameters = {"floor_height_m": 0., "feature_free": True}

    if name in ("ramp", "stairs"):
        parameters.update(ascent_start_x_m=start, ascent_end_x_m=top_begin,
                          landing_start_x_m=top_begin, landing_end_x_m=top_end,
                          descent_start_x_m=top_end, descent_end_x_m=end)
    if level != "standard":
        finish = max(6., end + .6)

    colours = {"ramp": (.51, .58, .36, 1.), "stairs": (.60, .48, .32, 1.),
               "rough": (.49, .48, .44, 1.), "hills": (.45, .56, .39, 1.),
               "flat": (.35, .4, .45, 1.)}
    for box in boxes:
        box.append_xml(world, colours[name])
    return TerrainProfile(name=name, level=level, difficulty=difficulty, seed=seed,
                          entry_x=start, exit_x=end, finish_x=finish,
                          width=width, parameters=parameters, _boxes=tuple(boxes))
