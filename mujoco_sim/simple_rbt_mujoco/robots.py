"""Small, ROS-independent robot selection and command contracts.

Limits are simulator input bounds, not proof of hardware or terrain performance.
Model modules load lazily so reading CLI/ROS names does not construct physics.
"""
from importlib import import_module
import math

ROBOT_NAMES = ("go2", "go2w", "diffdrive", "scout_mini", "fr_mini", "ridgeback", "ranger_mini")
CONTROLLER_NAMES = ("auto", "legacy", "himloco", "robot_lab", "rl_sar", "none",
                    "skid_steer", "ackermann", "mecanum", "swerve")
WHEEL_MODELS = {
    "scout_mini": ("scout_mini", "build_scout_mini_xml", "ScoutMiniController", "skid_steer"),
    "fr_mini": ("fr_mini", "build_fr_mini_xml", "FrMiniController", "ackermann"),
    "ridgeback": ("ridgeback", "build_ridgeback_xml", "RidgebackController", "mecanum"),
    "ranger_mini": ("ranger_mini", "build_ranger_mini_xml", "RangerMiniController", "swerve"),
}
WHEELED_ROBOTS = ("diffdrive", *WHEEL_MODELS)
NO_LATERAL_ROBOTS = ("diffdrive", "scout_mini", "fr_mini")
WHEEL_TERRAINS = ("flat", "ramp", "hills")
DEFAULT_CONTROLLERS = {"go2": "robot_lab", "go2w": "rl_sar", "diffdrive": "none",
                       **{name: spec[3] for name, spec in WHEEL_MODELS.items()}}
COMMAND_LIMITS = {name: (1., .6, 1.) for name in ROBOT_NAMES}
COMMAND_LIMITS.update(diffdrive=(1., 0., 1.), scout_mini=(1.5, 0., 1.), fr_mini=(1., 0., 1.))


def wheel_model(robot):
    module, builder, controller, name = WHEEL_MODELS[robot]
    loaded = import_module("." + module, __package__)
    return getattr(loaded, builder), getattr(loaded, controller), name


def command_is_supported(robot, vx, vy, wz):
    """Pure validation shared by direct commands and safe ROS rejection.

    Unknown ROS names retain a generic planar command contract, allowing named
    bridge instances independently of which concrete chassis a caller uses.
    """
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        return False
    if robot in NO_LATERAL_ROBOTS and vy != 0.:
        return False
    if robot == "fr_mini" and abs(vx) <= 1e-9 and wz != 0.:
        return False
    return True


def validate_terrain(robot, terrain):
    if robot in WHEELED_ROBOTS and terrain not in WHEEL_TERRAINS:
        raise ValueError(f"{robot} uses continuous surfaces; choose flat, ramp or hills")
