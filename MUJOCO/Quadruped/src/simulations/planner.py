import sys
import os

from simulations import gen_map
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import ompl.base as ob
import ompl.geometric as og
from core.manipulator.kinematics import MorphIManipulator
from gen_map import MapWorld
from typing import List, Tuple, Dict, Optional
      
class Planner:
    def __init__(
        self,
        map_world: 'MapWorld',
        manip_left: 'MorphIManipulator',
        manip_right: 'MorphIManipulator', 
        base_size: np.ndarray,
        vertical_size: np.ndarray,
        mobile2basearm: Dict[str, np.ndarray],
        basearm2verticalarm: Dict[str, np.ndarray],
        base_z:  float = 0.3,
        diamond_radii: Tuple[float, float, float] = (0.05, 0.08, 0.05),
        resolution: float = 0.1,
        timeout: float = 30.0
    ):
        if ob is None:
            raise RuntimeError("OMPL is required for planning but not installed.")
        
        self.map_ = map_world
        self.manip_left = manip_left
        self.manip_right = manip_right
        self.resolution = resolution
        self.timeout = timeout  
        self.base_z = base_z
        self.space = None
        self.si = None
        self.problem = None
        self.planner = None

        self.mobile2basearm = mobile2basearm
        self.basearm2verticalarm = basearm2verticalarm
        self.base_size = base_size
        self.vertical_size = vertical_size
        self.diamond_radii = diamond_radii
        self.min_lateral_dist_right = self.manip_right.min_lateral_dist
        self.min_lateral_dist_left = self.manip_left.min_lateral_dist

        self._define_problem_core()

    def _define_problem_core(self):
        try:
            min_bound, max_bound = self.map_.get_bounds()
        except ValueError as e:
            raise RuntimeError(f"Cannot plan: {e}")

        self.space = ob.RealVectorStateSpace(13)
        bounds = ob.RealVectorBounds(13)

        # Base
        bounds.setLow(0, min_bound[0]); bounds.setHigh(0, max_bound[0])
        bounds.setLow(1, min_bound[1]); bounds.setHigh(1, max_bound[1])
        bounds.setLow(2, -np.pi);      bounds.setHigh(2, np.pi)

        # Left arm
        left_bounds = [self.manip_left.bounds_h, self.manip_left.bounds_h,
                       self.manip_left.bounds_a, self.manip_left.bounds_theta,
                       self.manip_left.bounds_phi]
        for i, (low, high) in enumerate(left_bounds):
            bounds.setLow(3 + i, low); bounds.setHigh(3 + i, high)

        # Right arm
        right_bounds = [self.manip_right.bounds_h, self.manip_right.bounds_h,
                        self.manip_right.bounds_a, self.manip_right.bounds_theta,
                        self.manip_right.bounds_phi]
        for i, (low, high) in enumerate(right_bounds):
            bounds.setLow(8 + i, low); bounds.setHigh(8 + i, high)

        self.space.setBounds(bounds)
        self.si = ob.SpaceInformation(self.space)

        def is_state_valid(state):
            x, y, yaw = state[0], state[1], state[2]
            q_left = np.array([state[i] for i in range(3, 8)])
            q_right = np.array([state[i] for i in range(8, 13)])

            # Base bounds check
            if not (min_bound[0] <= x <= max_bound[0] and min_bound[1] <= y <= max_bound[1]):
                return False

            try:
                base_pose = (x, y, self.base_z, yaw)
                
                # Left arm EE
                pts_l = gen_map.fk_points_to_world(base_pose, self.mobile2basearm["left"], self.manip_left, q_left)
                ee_left = pts_l["ee"]
                if ee_left[0]**2 + ee_left[1]**2 < self.min_lateral_dist_left**2:
                    return False

                # Right arm EE
                pts_r = gen_map.fk_points_to_world(base_pose, self.mobile2basearm["right"], self.manip_right, q_right)
                ee_right = pts_r["ee"]
                if ee_right[0]**2 + ee_right[1]**2 < self.min_lateral_dist_right**2:
                    return False

            except Exception as e:
                print(f"FK error in lateral check: {e}", file=sys.stderr)
                return False

            # Collision check 
            try:
                edges = gen_map.get_all_robot_edges(
                    waypoints_yaw=[base_pose],
                    base_size=self.base_size,
                    vertical_size=self.vertical_size,
                    mobile2basearm=self.mobile2basearm,
                    basearm2verticalarm=self.basearm2verticalarm,
                    arm_left=self.manip_left,
                    arm_right=self.manip_right,
                    q_left=q_left,
                    q_right=q_right,
                    scale=1.0,
                    diamond_radii=self.diamond_radii,
                    edge_diagonal=True
                )
            except Exception as e:
                print(f"Edge generation error: {e}", file=sys.stderr)
                return False

            for p1, p2 in edges:
                if self.map_.is_los_blocked(p1, p2):
                    return False

            return True

        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))
        self.si.setup()

        self.problem = ob.ProblemDefinition(self.si)
        self.problem.setOptimizationObjective(ob.PathLengthOptimizationObjective(self.si))

        self.planner = og.RRTConnect(self.si)
        self.planner.setIntermediateStates(True)
        self.planner.setRange(self.resolution)
        self.planner.setProblemDefinition(self.problem)
        self.planner.setup()

    def solve(self, start_state: np.ndarray, goal_state: np.ndarray):
        if start_state.shape != (13,) or goal_state.shape != (13,):
            raise ValueError("States must be 13-dimensional arrays")

        start_ompl = ob.State(self.space)
        goal_ompl = ob.State(self.space)
        for i in range(13):
            start_ompl[i] = start_state[i]
            goal_ompl[i] = goal_state[i]

        if not self.si.satisfiesBounds(start_ompl.get()):
            print("Start state is out of bounds!")
            return None
        if not self.si.satisfiesBounds(goal_ompl.get()):
            print("Goal state is out of bounds!")
            return None

        if not self.si.isValid(start_ompl.get()):
            print("Start state is in collision or violates constraints!")
            return None
        if not self.si.isValid(goal_ompl.get()):
            print("Goal state is in collision or violates constraints!")
            return None

        print("Start and goal states are valid.")

        self.problem.setStartAndGoalStates(start_ompl, goal_ompl)

        print(f"Solving 13D whole-body planning (timeout={self.timeout}s)...")
        solved = self.planner.solve(self.timeout)

        if solved and self.problem.hasExactSolution():
            path = self.problem.getSolutionPath()
            print(f"Found path with {path.getStateCount()} states.")

            base_traj = [(s[0], s[1], s[2]) for s in path.getStates()]
            left_traj = [np.array([s[i] for i in range(3, 8)]) for s in path.getStates()]
            right_traj = [np.array([s[i] for i in range(8, 13)]) for s in path.getStates()]

            return base_traj, left_traj, right_traj
        else:
            print("No solution found.")
            return None
