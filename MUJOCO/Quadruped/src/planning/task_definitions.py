"""
Defines the waypoints and timing for each phase of the pick-and-place task.
Keeps mission-level logic separate from execution logic.
"""
import numpy as np
from planning.trajectory_planner import TrajectoryPlanner
from tasks.trajectory_task import TrajectoryTask
from tasks.hold_task import HoldTask
from tasks.drop_task import DropTask
from tasks.sequence import TaskSequence


def build_pick_and_place_sequence(
    initial_base_pose: np.ndarray,
    pick_position: np.ndarray,
    drop_position: np.ndarray,
) -> TaskSequence:
    """
    Build the full pick-and-place task sequence.

    Args:
        initial_base_pose: [x, y, yaw] of the robot at start.
        pick_position: [x, y, z] of the object to pick.
        drop_position: [x, y, yaw] of the drop zone.

    Returns:
        TaskSequence ready to execute.
    """
    planner = TrajectoryPlanner(num_points=50)
    sequence = TaskSequence()

    x0, y0, yaw0 = initial_base_pose

    # =========================================================================
    # PHASE 1: Drive base to pick area
    # =========================================================================
    base_waypoints_1 = np.array([
        [x0, y0, yaw0],
        [1.5, -2.5, yaw0 / 2],
        [1.5, -5.3, yaw0 / 3],
        [3.0, -6.2, 0.0],
    ])
    base_times_1 = [0.0, 5.0, 8.0, 11.0]
    base_traj_1 = planner.plan(
        waypoints=base_waypoints_1,
        times=base_times_1,
        n_coeffs=[4, 4, 4],
        derivatives=[2, 2, 2],
    )
    sequence.add(TrajectoryTask(
        name="drive_to_pick",
        trajectory=base_traj_1,
        target_type="base",
    ))

    # =========================================================================
    # PHASE 2a: Arm approach (move to above pick position)
    # =========================================================================
    arm_approach_waypoints = np.array([
        [pick_position[0], pick_position[1], pick_position[2], 0.0],
        [-0.33, -0.053, pick_position[2], 1.12],
        [-0.33, -0.053, -0.01, 1.12],
    ])
    arm_approach_times = [0.0, 2.0, 4.0]
    arm_approach_traj = planner.plan(
        waypoints=arm_approach_waypoints,
        times=arm_approach_times,
        n_coeffs=[5, 5, 5, 5],
        derivatives=[2, 2, 2, 2],
    )
    sequence.add(TrajectoryTask(
        name="arm_approach",
        trajectory=arm_approach_traj,
        target_type="arm_left",
        gripper_action="open",
        pitch_override=1.8,
    ))

    # =========================================================================
    # PHASE 2b: Hold gripper closed (grab the object)
    # =========================================================================
    sequence.add(HoldTask(
        name="grab_object",
        hold_duration=2.0,
        gripper_action="close",
        hold_position=arm_approach_traj.final_values()[:3],
        pitch_override=1.8,
        roll_override=arm_approach_traj.final_values()[3],
    ))

    # =========================================================================
    # PHASE 2c: Arm return to initial pose
    # =========================================================================
    arm_return_waypoints = np.array([
        arm_approach_traj.final_values()[:4],
        [pick_position[0], pick_position[1], pick_position[2], arm_approach_traj.final_values()[3]],
    ])
    arm_return_times = [0.0, 4.0]
    arm_return_traj = planner.plan(
        waypoints=arm_return_waypoints,
        times=arm_return_times,
        n_coeffs=[5, 5, 5, 5],
        derivatives=[2, 2, 2, 2],
    )
    sequence.add(TrajectoryTask(
        name="arm_return",
        trajectory=arm_return_traj,
        target_type="arm_left",
        gripper_action="close",
        pitch_override=1.8,
    ))

    # =========================================================================
    # PHASE 3: Drive base to drop zone (while arm prepares)
    # =========================================================================
    base_waypoints_2 = np.array([
        [3.0, -6.2, 0.0],
        [drop_position[0], drop_position[1], drop_position[2]],
    ])
    base_times_2 = [0.0, 4.0]
    base_traj_2 = planner.plan(
        waypoints=base_waypoints_2,
        times=base_times_2,
        n_coeffs=[4, 4, 4],
        derivatives=[2, 2, 2],
    )
    sequence.add(TrajectoryTask(
        name="drive_to_drop",
        trajectory=base_traj_2,
        target_type="base",
    ))

    # =========================================================================
    # PHASE 4: Arm moves to drop position (parallel with base driving)
    # =========================================================================
    arm_drive_waypoints = np.array([
        [pick_position[0], pick_position[1], pick_position[2],
         arm_return_traj.final_values()[3], 1.8],
        [-0.6, 0.0, 1.3, arm_return_traj.final_values()[3], 0.0],
    ])
    arm_drive_times = [0.0, 2.0]
    arm_drive_traj = planner.plan(
        waypoints=arm_drive_waypoints,
        times=arm_drive_times,
        n_coeffs=[5, 5, 5, 5, 5],
        derivatives=[2, 2, 2, 2, 2],
    )
    sequence.add(TrajectoryTask(
        name="arm_to_drop_position",
        trajectory=arm_drive_traj,
        target_type="arm_left",
    ))

    # =========================================================================
    # PHASE 5: Drop object (open gripper)
    # =========================================================================
    sequence.add(DropTask(
        name="drop_object",
        drop_position=np.array([-0.6, 0.0, 1.27]),
        open_duration=2.0,
        final_roll=arm_drive_traj.final_values()[3],
        final_pitch=arm_drive_traj.final_values()[4],
    ))

    return sequence