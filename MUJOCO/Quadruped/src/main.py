import sys
import os
import argparse
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from config import SimulationConfig, MotionProfileConfig, TaskConfig
from core.robot import ParallelRobot
from data.csv_source import CsvDataSource
from data.ws_source import WebSocketDataSource
from data.null_source import NullDataSource
from rendering.glfw_viewer import GlfwViewer
from rendering.cv_viewer import OpenCvViewer
from geometry.kinematics import ParallelArmKinematics
from utils.logger import setup_logger

logger = setup_logger("Main")


def main():
    parser = argparse.ArgumentParser(description="Run MuJoCo Parallel Robot Simulation")
    parser.add_argument("--run", choices=["glfw", "cv"], default="glfw")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--landmark-csv", type=str, default="processed_landmarks.csv")
    parser.add_argument("--playback-rate", type=float, default=1.0)
    parser.add_argument("--data-mode", choices=["csv", "ws"], default="csv")
    parser.add_argument("--ws-url", type=str, default="ws://localhost:9090")
    parser.add_argument("--target-mode", choices=["fixed", "closest"], default="fixed")
    parser.add_argument(
        "--control",
        choices=["landmark", "trajectory", "keyboard"],
        default="landmark",
        help="Control mode: landmark (vision), trajectory (autonomous), keyboard (manual)",
    )
    parser.add_argument(
        "--world",
        choices=["plain", "market"],
        default="market",
        help="World environment: 'plain' (plain_world.xml) or 'market' (market_world.xml)",
    )
    
    parser.add_argument("--pick-x", type=float, default=-0.33)
    parser.add_argument("--pick-y", type=float, default=-0.053)
    parser.add_argument("--pick-z", type=float, default=-0.01)
    parser.add_argument("--drop-x", type=float, default=3.0)
    parser.add_argument("--drop-y", type=float, default=-6.7)
    parser.add_argument("--drop-yaw", type=float, default=-1.5708)
    parser.add_argument("--profile", choices=["trapezoidal", "s_curve", "sinusoidal", "exponential"], default="trapezoidal")
    parser.add_argument("--max-vel", type=float, default=5.0)
    parser.add_argument("--max-accel", type=float, default=15.0)
    parser.add_argument("--max-jerk", type=float, default=80.0)

    args = parser.parse_args()

    if args.record and args.run != "cv":
        logger.warning("--record only works with --run cv. Ignoring.")
        args.record = False

    world_filename = f"{args.world}_world.xml"
    xml_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'env', world_filename))
    
    logger.info(f"Using world: {args.world} ({world_filename})")

    profile_config = MotionProfileConfig(
        profile_type=args.profile,
        max_vel=args.max_vel,
        max_accel=args.max_accel,
        max_jerk=args.max_jerk,
    )

    task_config = TaskConfig(
        pick_position=(args.pick_x, args.pick_y, args.pick_z),
        drop_position=(args.drop_x, args.drop_y, args.drop_yaw),
    )

    config = SimulationConfig(
        xml_path=xml_path,
        run_mode=args.run,
        record=args.record,
        data_mode=args.data_mode,
        ws_url=args.ws_url,
        landmark_csv=args.landmark_csv,
        playback_rate=args.playback_rate,
        target_mode=args.target_mode,
        control_mode=args.control,
        motion_profile=profile_config,
        task_config=task_config,
        world_name=args.world,
    )

    if config.control_mode == "landmark":
        if config.data_mode == "ws":
            data_source = WebSocketDataSource(config.ws_url)
        else:
            data_source = CsvDataSource(config.landmark_csv, config.playback_rate)
    else:
        data_source = NullDataSource()

    viewer = GlfwViewer() if config.run_mode == "glfw" else OpenCvViewer(record=config.record)
    kinematics = ParallelArmKinematics(np.zeros(3), np.zeros(3))

    sim = ParallelRobot(config, viewer, data_source, kinematics)
    data_source.start()
    sim.run()


if __name__ == "__main__":
    main()