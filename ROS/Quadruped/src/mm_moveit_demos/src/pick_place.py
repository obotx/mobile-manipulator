#!/usr/bin/env python3
import subprocess
import sys
import time

MAX_RETRIES = 5         
RETRY_DELAY_SEC = 1.0   
FAILURE_KEYWORDS = {"[ERROR]", "FAILED", "aborted", "MoveGroupInterface::plan() failed"}

def run_single_step(cmd):
    process = None
    try:
        process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1
        )
        step_failed = False

        for line in process.stdout:
            print(line, end="")
            if any(kw in line for kw in FAILURE_KEYWORDS):
                step_failed = True

        process.wait()

        if process.returncode != 0 or step_failed:
            return False
        return True

    except KeyboardInterrupt:
        print("\nSequence interrupted by user.")
        if process:
            try: process.terminate()
            except: pass
        sys.exit(0)
    except Exception as e:
        print(f"\n Unexpected error: {e}")
        return False

def main():
    steps = [
        # === PICK ===
        ["ros2", "run", "mm_nav2", "move_to_obj.py"],

        ["ros2", "run", "mm_moveit_demos", "move_arm_pose", "--ros-args",
         "-p", "arm_side:=left", "-p", "target_x:=-2.5", "-p", "target_y:=-1.0", "-p", "target_z:=1.1"],

        ["ros2", "run", "mm_moveit_demos", "move_arm_pose", "--ros-args",
         "-p", "arm_side:=left", "-p", "target_x:=-2.5", "-p", "target_y:=-1.2", "-p", "target_z:=1.1"],

        ["ros2", "run", "mm_moveit_demos", "move_group_state", "--ros-args",
         "-p", "group_name:=gripper_left", "-p", "state_name:=close"],

        ["ros2", "run", "mm_moveit_demos", "move_arm_pose", "--ros-args",
         "-p", "arm_side:=left", "-p", "target_x:=-2.5", "-p", "target_y:=-1.2", "-p", "target_z:=1.2"],

        ["ros2", "run", "mm_moveit_demos", "move_arm_pose", "--ros-args",
         "-p", "arm_side:=left", "-p", "target_x:=-2.5", "-p", "target_y:=-1.0", "-p", "target_z:=1.2"],

        ["ros2", "run", "mm_moveit_demos", "move_group_state", "--ros-args",
         "-p", "group_name:=left_arm", "-p", "state_name:=home_left_arm"],

        # === PLACE ===
        ["ros2", "run", "mm_nav2", "move_to_target.py", "--ros-args",
         "-p", "target_x:=2.5", "-p", "target_y:=1.2", "-p", "face_to_target:=true"],

        ["ros2", "run", "mm_moveit_demos", "move_arm_pose", "--ros-args",
         "-p", "arm_side:=left", "-p", "target_x:=2.5", "-p", "target_y:=1.0", "-p", "target_z:=1.2"],

        ["ros2", "run", "mm_moveit_demos", "move_arm_pose", "--ros-args",
         "-p", "arm_side:=left", "-p", "target_x:=2.5", "-p", "target_y:=1.2", "-p", "target_z:=1.2"],

        ["ros2", "run", "mm_moveit_demos", "move_arm_pose", "--ros-args",
         "-p", "arm_side:=left", "-p", "target_x:=2.5", "-p", "target_y:=1.2", "-p", "target_z:=1.15"],

        ["ros2", "run", "mm_moveit_demos", "move_group_state", "--ros-args",
         "-p", "group_name:=gripper_left", "-p", "state_name:=open"],

        ["ros2", "run", "mm_moveit_demos", "move_arm_pose", "--ros-args",
         "-p", "arm_side:=left", "-p", "target_x:=2.5", "-p", "target_y:=1.0", "-p", "target_z:=1.15"],

        ["ros2", "run", "mm_moveit_demos", "move_group_state", "--ros-args",
         "-p", "group_name:=left_arm", "-p", "state_name:=home_left_arm"],

        ["ros2", "run", "mm_nav2", "move_to_target.py", "--ros-args",
         "-p", "target_x:=0.0", "-p", "target_y:=0.0", "-p", "face_to_target:=false"],
    ]

    print("Starting 'Pick & Place' sequence...")
    total_steps = len(steps)

    for i, cmd in enumerate(steps, 1):
        success = False
        for attempt in range(1, MAX_RETRIES + 1):
            print(f"[Step {i}/{total_steps}] (Attempt {attempt}/{MAX_RETRIES})")

            if run_single_step(cmd):
                success = True
                print("Step completed successfully.")
                break

            if attempt < MAX_RETRIES:
                print(f"Step {i} failed. Retrying in {RETRY_DELAY_SEC} seconds...")
                time.sleep(RETRY_DELAY_SEC)

        if not success:
            print(f"\nStep {i} failed after {MAX_RETRIES} attempts.")
            print("Aborting entire sequence.")
            sys.exit(1)

        time.sleep(1.0)

    print("\n'Pick & Place' sequence completed successfully.")

if __name__ == '__main__':
    main()