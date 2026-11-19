"""Shared builders for the TATARO Lite6 + Android-phone LeRobot scripts."""

import argparse

from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.processor import (
    RobotProcessorPipeline,
    make_default_processors,
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot_robot_lite6 import Lite6, Lite6Config
from lerobot_robot_lite6.phone_processor import Lite6PhoneToEE
from lerobot_robot_lite6.webxr_phone import Lite6WebXRPhone, Lite6WebXRPhoneConfig


def add_common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--ip", default="192.168.1.150", help="Lite6 controller IP")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--scale", type=float, default=1.0, help="TCP metres per phone metre")
    p.add_argument("--no-rotation", action="store_true", help="translate only, keep tool orientation fixed")
    p.add_argument("--max-speed", type=float, default=0.15, help="TCP speed limit [m/s]")
    p.add_argument("--smoothing", type=float, default=0.10, help="tracking time constant [s]; higher = smoother but laggier")
    p.add_argument("--max-acc", type=float, default=1.0, help="TCP acceleration limit [m/s^2]")
    p.add_argument("--no-cameras", action="store_true", help="arm only (teleop testing)")
    p.add_argument("--display", action="store_true", help="show cameras/state in Rerun")


def make_robot(args) -> Lite6:
    cfg = Lite6Config(
        id="tataro_lite6",
        ip=args.ip,
        max_lin_speed_m_s=args.max_speed,
        smoothing_time_s=args.smoothing,
        max_lin_acc_m_s2=args.max_acc,
    )
    if args.no_cameras:
        cfg.cameras = {}
    return Lite6(cfg)


def make_phone() -> Lite6WebXRPhone:
    return Lite6WebXRPhone(Lite6WebXRPhoneConfig())


def make_processors(args):
    """(teleop_action_processor, robot_action_processor, robot_observation_processor)."""
    teleop_action_processor = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        steps=[Lite6PhoneToEE(position_scale=args.scale, use_rotation=not args.no_rotation)],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )
    _, robot_action_processor, robot_observation_processor = make_default_processors()  # identities
    return teleop_action_processor, robot_action_processor, robot_observation_processor
