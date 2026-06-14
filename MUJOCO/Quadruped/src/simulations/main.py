import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import os
import argparse
from core.robot.morph_i import ParallelRobot

def main():
    parser = argparse.ArgumentParser(description="Run MuJoCo Parallel Robot Simulation")
    parser.add_argument("--run", choices=["glfw", "cv"], default="glfw")
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args()

    if args.record and args.run != "cv":
        print("Warning: --record only works with --run cv")
        args.record = False

    xml_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../env', 'market_world_plain.xml'))
    sim = ParallelRobot(xml_path, args.run, args.record)

    if args.run == "glfw":
        sim.run_glfw()
    else:
        sim.run_cv()

if __name__ == "__main__":
    main()