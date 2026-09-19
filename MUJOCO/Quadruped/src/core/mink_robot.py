import mujoco
import numpy as np
import mink
from typing import Dict, List
from config import TeleopConfig
from scipy.spatial.transform import Rotation
from utils.logger import log_robot, log_error, log_debug

class MinkRobotInterface:
    def __init__(self, config: TeleopConfig):
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(config.xml_path.as_posix())
        self.data = mujoco.MjData(self.model)
        self.configuration = mink.Configuration(self.model)

        self.motor_ctrl_ids: np.ndarray = np.array([])
        self.motor_qpos_adrs: np.ndarray = np.array([])
        self.finger_ctrl_ids: Dict = {}
        self.finger_qpos_adrs: Dict = {}
        self.finger_body_ids: Dict = {
            "left": {"a": [], "b": [], "c": []},
            "right": {"a": [], "b": [], "c": []}
        }

        self.finger_close_targets = {"joint1": 0.33, "joint2": 0.0, "joint3": 0.0}
        self.finger_open_targets = {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0}

        self.tasks: List = []
        self.limits: List = []
        self.left_end_effector_task = None
        self.right_end_effector_task = None
        self.posture_task = None

        self.left_mocap_id = self.model.body("left_ik_target").mocapid[0]
        self.right_mocap_id = self.model.body("right_ik_target").mocapid[0]

        self._setup_motors()
        self._setup_fingers()
        self._setup_finger_body_ids()
        self._setup_tasks_and_limits()

        log_robot(f"Initialized with {len(self.motor_ctrl_ids)} motors")

    def _setup_motors(self):
        motor_joint_names = [
            f"{self.config.robot_prefix}_{arm}_{motor}"
            for arm in self.config.arm_prefixes
            for motor in self.config.arm_motors
        ]
        ctrl_ids = []
        qpos_adrs = []
        for name in motor_joint_names:
            try:
                jnt_id = self.model.joint(name).id
            except KeyError:
                continue
            actuator_indices = np.where(self.model.actuator_trnid[:, 0] == jnt_id)[0]
            if len(actuator_indices) > 0:
                ctrl_ids.append(actuator_indices[0])
                qpos_adrs.append(self.model.jnt_qposadr[jnt_id])
        self.motor_ctrl_ids = np.array(ctrl_ids)
        self.motor_qpos_adrs = np.array(qpos_adrs)

    def _setup_fingers(self):
        self.finger_ctrl_ids = {
            side: {letter: {"joint1": [], "joint2": [], "joint3": []} for letter in "abc"}
            for side in ("left", "right")
        }
        self.finger_qpos_adrs = {
            side: {letter: {"joint1": [], "joint2": [], "joint3": []} for letter in "abc"}
            for side in ("left", "right")
        }

        for i in range(self.model.nu):
            jnt_id = self.model.actuator_trnid[i, 0]
            jnt_name = self.model.joint(jnt_id).name
            if "finger" not in jnt_name:
                continue
            arm_side = "left" if "left" in jnt_name else "right"
            qpos_adr = self.model.jnt_qposadr[jnt_id]

            if "finger_a" in jnt_name:
                letter = "a"
            elif "finger_b" in jnt_name:
                letter = "b"
            elif "finger_c" in jnt_name:
                letter = "c"
            else:
                continue

            if jnt_name.endswith("joint1"):
                key = "joint1"
            elif jnt_name.endswith("joint2"):
                key = "joint2"
            elif jnt_name.endswith("joint3"):
                key = "joint3"
            else:
                continue

            self.finger_ctrl_ids[arm_side][letter][key].append(i)
            self.finger_qpos_adrs[arm_side][letter][key].append(qpos_adr)

        for side in self.finger_ctrl_ids:
            for letter in self.finger_ctrl_ids[side]:
                for key in self.finger_ctrl_ids[side][letter]:
                    self.finger_ctrl_ids[side][letter][key] = np.array(self.finger_ctrl_ids[side][letter][key])
                    self.finger_qpos_adrs[side][letter][key] = np.array(self.finger_qpos_adrs[side][letter][key])

    def _setup_finger_body_ids(self):
        for i in range(1, self.model.nbody):
            name = self.model.body(i).name
            if not name:
                continue
            side = "left" if "left" in name else "right" if "right" in name else None
            if not side:
                continue
            if "finger_a" in name and any(p in name for p in ("proximal", "middle", "distal")):
                self.finger_body_ids[side]["a"].append(i)
            elif "finger_b" in name and any(p in name for p in ("proximal", "middle", "distal")):
                self.finger_body_ids[side]["b"].append(i)
            elif "finger_c" in name and any(p in name for p in ("proximal", "middle", "distal")):
                self.finger_body_ids[side]["c"].append(i)

        for side in self.finger_body_ids:
            for letter in self.finger_body_ids[side]:
                self.finger_body_ids[side][letter] = np.array(self.finger_body_ids[side][letter], dtype=int)

    def _setup_tasks_and_limits(self):
        p = self.config.robot_prefix

        self.left_end_effector_task = mink.FrameTask(
            frame_name=f"{p}_{self.config.arm_prefixes[0]}_ee_site", frame_type="site",
            position_cost=5.0,
            orientation_cost=5.0,
            lm_damping=1e-2,
        )
        self.right_end_effector_task = mink.FrameTask(
            frame_name=f"{p}_{self.config.arm_prefixes[1]}_ee_site", frame_type="site",
            position_cost=5.0,
            orientation_cost=5.0,
            lm_damping=1e-2,
        )

        self.posture_task = mink.PostureTask(model=self.model, cost=1e-3, lm_damping=1e-2)
        equality_task = mink.EqualityConstraintTask(model=self.model, cost=5000.0, gain=1.0, lm_damping=1e-3)

        self.tasks = [
            self.left_end_effector_task,
            self.right_end_effector_task,
            self.posture_task,
            equality_task,
        ]

        left_arm_geoms = mink.get_subtree_geom_ids(self.model, self.model.body(f"{p}_{self.config.arm_prefixes[0]}_arm_base").id)
        right_arm_geoms = mink.get_subtree_geom_ids(self.model, self.model.body(f"{p}_{self.config.arm_prefixes[1]}_arm_base").id)
        base_geoms = mink.get_body_geom_ids(self.model, self.model.body(f"{p}_base").id)
        floor_geom = mink.get_body_geom_ids(self.model, self.model.body("world_floor").id)

        collision_limit = mink.CollisionAvoidanceLimit(
            model=self.model,
            geom_pairs=[
                (floor_geom, left_arm_geoms),
                (floor_geom, right_arm_geoms),
                (left_arm_geoms, right_arm_geoms),
                (left_arm_geoms, base_geoms),
                (right_arm_geoms, base_geoms),
            ],
            gain=0.2,
            minimum_distance_from_collisions=0.02,
            collision_detection_distance=0.25,
            bound_relaxation=0.01,
            broadphase=True,
        )

        max_velocities = {
            f"{p}_{arm}_{motor}": vel
            for arm in self.config.arm_prefixes
            for motor, vel in self.config.arm_motors_max_vel.items()
        }
        velocity_limit = mink.VelocityLimit(self.model, max_velocities)

        self.limits = [
            mink.ConfigurationLimit(model=self.configuration.model),
            # collision_limit,  # Uncomment if you want collision avoidance enabled
            velocity_limit,
        ]

    def reset(self):
        if self.model.nkey > 0 and "home" in [self.model.key(i).name for i in range(self.model.nkey)]:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        else:
            mujoco.mj_resetData(self.model, self.data)

        self.configuration.update(self.data.qpos)
        self.posture_task.set_target_from_configuration(self.configuration)

        mujoco.mj_forward(self.model, self.data)
        p = self.config.robot_prefix
        mink.move_mocap_to_frame(self.model, self.data, "left_ik_target", f"{p}_left_ee_site", "site")
        mink.move_mocap_to_frame(self.model, self.data, "right_ik_target", f"{p}_right_ee_site", "site")

        log_robot("Reset to home keyframe, IK targets synced")

    def get_gripper_contact_forces(self) -> Dict:
        forces = {'left': {'a': 0.0, 'b': 0.0, 'c': 0.0},
                  'right': {'a': 0.0, 'b': 0.0, 'c': 0.0}}
        cfrc = self.data.cfrc_ext
        for side in ("left", "right"):
            for letter, ids in self.finger_body_ids[side].items():
                if len(ids) > 0:
                    forces[side][letter] = np.sum(np.linalg.norm(cfrc[ids, 3:6], axis=1))
        return forces

    def get_gripper_roll_xy(self, arm_idx: int) -> np.ndarray:
        body_name = f"{self.config.robot_prefix}_{self.config.arm_prefixes[arm_idx]}_gripper_roll"
        try:
            body_id = self.model.body(body_name).id
            return self.data.xpos[body_id, :2].copy()
        except KeyError:
            return np.array([0.0, 0.0])

    def get_robot_arm_y(self, side: str) -> float:
        name = self.config.robot_arm_left_name if side == "left" else self.config.robot_arm_right_name
        try:
            body_id = self.model.body(name).id
            return self.data.xpos[body_id, 1]
        except KeyError:
            return 0.0

    def get_mocap_pos(self, side: str) -> np.ndarray:
        mocap_id = self.left_mocap_id if side == "left" else self.right_mocap_id
        return self.data.mocap_pos[mocap_id].copy()

    def sync_mocap_to_ee(self, side: str):
        p = self.config.robot_prefix
        site_name = f"{p}_{side}_ee_site"
        try:
            site_id = self.model.site(site_name).id
            self.set_mocap_pos(side, self.data.site_xpos[site_id].copy())
            rot_mat = self.data.site_xmat[site_id].reshape(3, 3)
            quat_xyzw = Rotation.from_matrix(rot_mat).as_quat()
            quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
            self.set_mocap_quat(side, quat_wxyz)
        except KeyError:
            log_error(f"Site {site_name} not found", tag="ROBOT")

    def set_mocap_pos(self, side: str, pos: np.ndarray):
        mocap_id = self.left_mocap_id if side == "left" else self.right_mocap_id
        self.data.mocap_pos[mocap_id] = np.asarray(pos, dtype=np.float64).flatten()

    def set_mocap_quat(self, side: str, quat: np.ndarray):
        """Set mocap quaternion. Accepts any shape and flattens to 1D [w,x,y,z]."""
        mocap_id = self.left_mocap_id if side == "left" else self.right_mocap_id
        self.data.mocap_quat[mocap_id] = np.asarray(quat, dtype=np.float64).flatten()

    def set_ee_targets(self):
        T_left = mink.SE3.from_mocap_name(self.model, self.data, "left_ik_target")
        self.left_end_effector_task.set_target(T_left)
        T_right = mink.SE3.from_mocap_name(self.model, self.data, "right_ik_target")
        self.right_end_effector_task.set_target(T_right)

    def solve_and_step(self, dt: float):
        try:
            vel = mink.solve_ik(
                self.configuration, self.tasks, dt, self.config.solver,
                safety_break=False, damping=self.config.ik_damping, limits=self.limits
            )
        except Exception as e:
            log_debug(f"IK solve failed: {e}", tag="ROBOT")
            return

        self.configuration.integrate_inplace(vel, dt)
        self.data.ctrl[self.motor_ctrl_ids] = self.configuration.q[self.motor_qpos_adrs]
        mujoco.mj_step(self.model, self.data, nstep=self.config.sim_steps_per_ctrl)
        self.configuration.update(self.data.qpos)

    def apply_gripper_commands(self, commands: Dict):
        if not commands:
            return

        new_posture_target = self.configuration.q.copy()
        all_forces = self.get_gripper_contact_forces()

        for (side, letter, joint), target_val in commands.items():
            is_closing = np.isclose(target_val, self.finger_close_targets[joint], atol=1e-3)
            threshold = self.config.force_threshold_left if side == "left" else self.config.force_threshold_right
            finger_force = all_forces[side][letter]

            if is_closing and finger_force > threshold:
                current_qpos = self.data.qpos[self.finger_qpos_adrs[side][letter][joint]]
                self.data.ctrl[self.finger_ctrl_ids[side][letter][joint]] = current_qpos
                new_posture_target[self.finger_qpos_adrs[side][letter][joint]] = current_qpos
            else:
                self.data.ctrl[self.finger_ctrl_ids[side][letter][joint]] = target_val
                new_posture_target[self.finger_qpos_adrs[side][letter][joint]] = target_val

        self.posture_task.target = new_posture_target
