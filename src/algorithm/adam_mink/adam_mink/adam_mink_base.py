#!/usr/bin/env python3
"""Base class for Adam robot inverse kinematics using Mink solver."""

from abc import ABC, abstractmethod
from threading import Lock, Thread
from typing import Dict, List, Tuple, Union

import mink
import mujoco as mj
import mujoco.viewer as mjv
import numpy as np
import rclpy
import tf2_ros
import yaml
from adam_mink.utils import draw_frame, quat_mul
from adam_mink.constants import ALL_FINGER
from loop_rate_limiters import RateLimiter
from pydantic import BaseModel
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import time

# Constants
DEFAULT_TIMER_PERIOD = 0.01  # 100 Hz
DEFAULT_JOINT_STATE_QUEUE_SIZE = 10
DEFAULT_MUJOCO_VIEWER_FREQUENCY = 200.0
DEFAULT_IK_SOLVER = "daqp"
DEFAULT_IK_DAMPING = 3e-1
DEFAULT_IK_ITER_MAX = 3
DEFAULT_IK_ERROR_THRESHOLD = 0.001
DEFAULT_IK_ITER_WARN_THRESHOLD = 0.8
DEFAULT_FINGER_POSITION = 1000.0
DEFAULT_FRAME_SIZE = 0.1
ROOT_POSE_NUM = 7  # x, y, z, qw, qx, qy, qz


class IkConfig(BaseModel):
    """Configuration for inverse kinematics task."""

    adam_link_name: str
    bone_name: str
    position_cost: float
    orientation_cost: float
    pos_offset: tuple[float, float, float]  # (x, y, z)
    rot_offset: tuple[float, float, float, float]  # Quaternion (w, x, y, z)


class Collision(BaseModel):
    """Configuration for collision avoidance."""

    collision1: list[str]
    collision2: list[str]
    min_distance: float = 0.0
    detection_distance: float = 0.0


class AdamMinkConfig(BaseModel):
    """Main configuration for Adam Mink IK solver."""

    ik_cfg: list[IkConfig] = []
    collision_cfg: list[Collision] = []
    velocity_limit: Dict[str, float] = {}
    human_scale_table: Dict[str, float] = {}


class AdamMinkBase(Node, ABC):
    """Base class for Adam robot IK using Mink solver with TF2 transforms."""

    def __init__(self, node_name: str) -> None:
        """Initialize the base node.

        Args:
            node_name: Name of the ROS2 node.
        """
        super().__init__(node_name)
        
        # Declare parameters with defaults
        self.declare_parameter("adam_model_path", "")
        self.declare_parameter("adam_mink_cfg", "")
        self.declare_parameter("mujoco_sim", False)
        self.declare_parameter("ik_iter_max", DEFAULT_IK_ITER_MAX)
        self.declare_parameter("ik_damping", DEFAULT_IK_DAMPING)
        self.declare_parameter("ik_error_threshold", DEFAULT_IK_ERROR_THRESHOLD)
        self.declare_parameter("ik_solver", DEFAULT_IK_SOLVER)
        # Get parameter values
        adam_mink_cfg_path = self.get_parameter("adam_mink_cfg").value
        adam_model_path = self.get_parameter("adam_model_path").value
        mujoco_sim = self.get_parameter("mujoco_sim").value
        self.ik_iter_max = self.get_parameter("ik_iter_max").value
        self.ik_damping = self.get_parameter("ik_damping").value
        self.ik_error_threshold = self.get_parameter("ik_error_threshold").value
        self.ik_solver = self.get_parameter("ik_solver").value

        # Validate required parameters
        if not adam_model_path:
            raise ValueError(
                "Required parameter 'adam_model_path' is not set or is empty"
            )
        if not adam_mink_cfg_path:
            raise ValueError(
                "Required parameter 'adam_mink_cfg' is not set or is empty"
            )
        
        # Validate parameter ranges
        if self.ik_iter_max <= 0:
            raise ValueError("ik_iter_max must be positive")
        if self.ik_damping < 0:
            raise ValueError("ik_damping must be non-negative")
        if self.ik_error_threshold < 0:
            raise ValueError("ik_error_threshold must be non-negative")

        self.get_logger().info(f"adam_mink_cfg_path: {adam_mink_cfg_path}")
        self.get_logger().info(f"adam_model_path: {adam_model_path}")

        # Thread synchronization lock for shared data
        self._data_lock = Lock()

        self.base_frame = "world"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", DEFAULT_JOINT_STATE_QUEUE_SIZE
        )
        self.timer = self.create_timer(DEFAULT_TIMER_PERIOD, self.cb_transform)

        # Get bone frames from subclass (bones to track from TF)
        self.bone_frames = self.get_bone_frames()

        # Load model with error handling
        try:
            self.model = mj.MjModel.from_xml_path(adam_model_path)
        except Exception as e:
            self.get_logger().fatal(
                f"Failed to load MuJoCo model from {adam_model_path}: {e}"
            )
            raise

        # Initialize configuration to get all bone names
        self.configuration = mink.Configuration(self.model)
        
        # Load config file with error handling
        try:
            with open(adam_mink_cfg_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            if data is None:
                raise ValueError(f"Config file {adam_mink_cfg_path} is empty or invalid")
        except FileNotFoundError:
            self.get_logger().fatal(f"Config file not found: {adam_mink_cfg_path}")
            raise
        except yaml.YAMLError as e:
            self.get_logger().fatal(f"Failed to parse YAML config: {e}")
            raise
        
        try:
            self.adam_mink_cfg = AdamMinkConfig.model_validate(data)
        except Exception as e:
            self.get_logger().fatal(f"Failed to validate config: {e}")
            raise
        
        # Validate config completeness
        if not self.adam_mink_cfg.ik_cfg:
            raise ValueError("IK configuration is empty")
        
        # Create bone_name -> cfg mapping to avoid nested loop
        self.bone_name_to_cfg = {cfg.bone_name: cfg for cfg in self.adam_mink_cfg.ik_cfg}

        # Warn about missing bone frames in config
        for bone in self.bone_frames:
            if not any(cfg.bone_name == bone for cfg in self.adam_mink_cfg.ik_cfg):
                self.get_logger().warn(
                    f"Bone frame '{bone}' not found in IK config"
                )
        
        self.get_logger().info(f"Loaded config: {self.adam_mink_cfg}")

        # Initialize mocap_data with all bones from config (not just tracked ones)
        self.mocap_data = self._initialize_mocap_data()
        self.robot_motor_names = {}
        for i in range(self.model.nu):  # 'nu' is the number of actuators (motors)
            motor_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i)
            self.robot_motor_names[f"dof_pos/{motor_name}"] = i
            self.get_logger().info(f"Actuator {i}: {motor_name}")

        self.finger_joint_num = len(ALL_FINGER)
        self.all_joint = {
            name: i
            for i, name in enumerate(
                list(self.robot_motor_names.keys()) + list(ALL_FINGER)
            )
        }
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = (
            [f"root_pos/{axis}" for axis in ("x", "y", "z")]
            + [f"root_quat/{r}" for r in ("w", "x", "y", "z")]
            + list(self.all_joint.keys())
        )
        self.joint_state_msg.position = [DEFAULT_FINGER_POSITION] * (
            ROOT_POSE_NUM + len(self.all_joint)
        )

        self.tasks = self._create_ik_tasks()
        self.limits = self._create_ik_limits()

        # Start Mujoco viewer thread
        if mujoco_sim:
            mj_t = Thread(target=self.mujoco_t, daemon=True)
            mj_t.start()

        self.get_logger().info(f"{node_name} node started")

    @abstractmethod
    def get_bone_frames(self) -> list[str]:
        """Get the list of bone frame names to track.

        Returns:
            List of bone frame names.
        """
        pass

    def _initialize_mocap_data(self) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        """Initialize mocap data dictionary with all bones from config."""
        mocap_data = {}
        # Initialize all bones from config (some may not be tracked from TF)
        for cfg in self.adam_mink_cfg.ik_cfg:
            if cfg.bone_name not in mocap_data:
                mocap_data[cfg.bone_name] = (
                    np.array([0, 0, 0.0]),
                    np.array([1, 0, 0, 0]),
                )
        return mocap_data

    def _create_ik_tasks(self) -> list[mink.FrameTask]:
        """Create IK tasks from configuration."""
        return [
            mink.FrameTask(
                frame_name=cfg.adam_link_name,
                frame_type="body",
                position_cost=cfg.position_cost,
                orientation_cost=cfg.orientation_cost,
                lm_damping=1.0,
            )
            for cfg in self.adam_mink_cfg.ik_cfg
        ]

    def _create_ik_limits(
        self,
    ) -> List[
        Union[
            mink.ConfigurationLimit,
            mink.CollisionAvoidanceLimit,
            mink.VelocityLimit,
        ]
    ]:
        """Create IK limits including collision avoidance and velocity limits."""
        collision_avoidance_limit = [
            mink.CollisionAvoidanceLimit(
                model=self.model,
                geom_pairs=[(cfg.collision1, cfg.collision2)],  # type: ignore
                minimum_distance_from_collisions=cfg.min_distance,
                collision_detection_distance=cfg.detection_distance,
            )
            for cfg in self.adam_mink_cfg.collision_cfg
        ]
        limits: List[
            Union[
                mink.ConfigurationLimit,
                mink.CollisionAvoidanceLimit,
                mink.VelocityLimit,
            ]
        ] = [mink.ConfigurationLimit(self.model)]
        limits.extend(collision_avoidance_limit)

        velocity_limit = mink.VelocityLimit(
            self.model, self.adam_mink_cfg.velocity_limit
        )
        limits.append(velocity_limit)
        return limits

    def cb_transform(self) -> None:
        """Check TF transforms and update IK solution."""
        time_start = time.time()
        if not self._update_mocap_data():
            return
        self.scale_mocap_data()
        # time_update_mocap = time.time()
        # self.get_logger().info(f"Update mocap data time: {time_update_mocap - time_start:.6f} s")
        self.offset_mocap_data()
        # time_offset = time.time()
        # self.get_logger().info(f"Offset mocap data time: {time_offset - time_update_mocap:.6f} s")
        self._update_ik_targets()
        # time_update = time.time()
        # self.get_logger().info(f"Update IK targets time: {time_update - time_offset:.6f} s")
        self._solve_ik()
        # time_solve = time.time()
        # self.get_logger().info(f"Solve IK time: {time_solve - time_update:.6f} s")
        self._publish_joint_states()
        time_publish = time.time()
        # self.get_logger().info(f"Publish joint states time: {time_publish - time_solve:.6f} s")
        # self.get_logger().info(f"all time: {time_publish - time_start:.6f} s")

    def _update_mocap_data(self) -> bool:
        """Update mocap data from TF transforms."""
        mocap_data_update = {}

        try:
            for bone in self.bone_frames:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, bone, rclpy.time.Time()
                )
                mocap_data_update[bone] = (
                    np.array(
                        [
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z,
                        ]
                    ),
                    np.array(
                        [
                            transform.transform.rotation.w,
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                        ]
                    ),
                )
        except tf2_ros.LookupException:
            return False
        except tf2_ros.ConnectivityException:
            self.get_logger().warn(f"Transform connectivity exception: world -> {bone}")
            return False
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn(
                f"Transform extrapolation exception: world -> {bone}"
            )
            return False

        # Update shared data with lock protection
        with self._data_lock:
            self.mocap_data.update(mocap_data_update)
        return True

    def _update_ik_targets(self) -> None:
        """Update IK task targets from mocap data."""
        # Access shared data with lock protection, but only read needed keys
        with self._data_lock:
            for task, cfg in zip(self.tasks, self.adam_mink_cfg.ik_cfg):
                if cfg.bone_name in self.mocap_data:
                    pos, rot = self.mocap_data[cfg.bone_name]
                    task.set_target(
                        mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos)
                    )
                else:
                    self.get_logger().warn(
                        f"Bone '{cfg.bone_name}' not found in mocap_data"
                    )

    def _solve_ik(self) -> None:
        """Solve inverse kinematics iteratively."""
        try:
            curr_error = self._compute_ik_error()
            dt = self.configuration.model.opt.timestep
            num_iter = 0
            next_error = curr_error

            while num_iter < self.ik_iter_max:
                try:
                    vel = mink.solve_ik(
                        configuration=self.configuration,
                        tasks=self.tasks,
                        dt=dt,
                        solver=self.ik_solver,
                        damping=self.ik_damping,
                        limits=self.limits,
                    )
                except Exception as e:
                    self.get_logger().error(
                        f"IK solve failed at iteration {num_iter}: {e}"
                    )
                    # Use zero velocity to maintain current configuration
                    break

                self.configuration.integrate_inplace(vel, dt)
                next_error = self._compute_ik_error()

                if curr_error - next_error <= self.ik_error_threshold:
                    break

                curr_error = next_error
                num_iter += 1

            if num_iter > self.ik_iter_max * DEFAULT_IK_ITER_WARN_THRESHOLD:
                self.get_logger().debug(
                    f"IK iteration count is high: {num_iter}/{self.ik_iter_max}, "
                    f"error: {next_error:.6f}"
                )
        except Exception as e:
            self.get_logger().error(
                f"Critical error in IK solving: {e}", exc_info=True
            )

    def _publish_joint_states(self) -> None:
        """Publish joint states to ROS topic."""
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = (
            list(self.configuration.data.qpos[:])
            + [DEFAULT_FINGER_POSITION] * self.finger_joint_num
        )

        # Update hand control (subclass can override)
        self.update_joint_states()

        self.joint_state_pub.publish(self.joint_state_msg)

    def update_joint_states(self) -> None:
        """Update joint states. Override in subclass if needed."""
        pass

    def scale_mocap_data(self) -> None:
        """Scale mocap data based on human scale table."""
        # Access shared data with lock protection
        if not self.adam_mink_cfg.human_scale_table:
            return
        with self._data_lock:
            mocap_data_copy = self.mocap_data.copy()

        human_data_local = {}
        root_pos, root_quat = mocap_data_copy["Hips"]

        # scale root
        scaled_root_pos = self.adam_mink_cfg.human_scale_table["Hips"] * root_pos

        # scale other body parts in local frame
        for body_name in mocap_data_copy.keys():
            if body_name not in self.adam_mink_cfg.human_scale_table.keys():
                continue
            if body_name == "Hips":
                continue
            else:
                # transform to local frame (only position)
                human_data_local[body_name] = (
                    mocap_data_copy[body_name][0] - root_pos
                ) * self.adam_mink_cfg.human_scale_table[body_name]

        # transform the human data back to the global frame
        human_data_global = {"Hips": (scaled_root_pos, root_quat)}
        for body_name in human_data_local.keys():
            human_data_global[body_name] = (
                human_data_local[body_name] + scaled_root_pos,
                mocap_data_copy[body_name][1],
            )

        with self._data_lock:
            self.mocap_data = human_data_global

    def offset_mocap_data(self) -> None:
        """Apply position and rotation offsets to mocap data."""
        # Access shared data with lock protection
        with self._data_lock:
            mocap_data_copy = self.mocap_data.copy()

        offset_human_data: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for mocap_data_name, (pos, rot) in mocap_data_copy.items():
            offset_human_data[mocap_data_name] = (pos.copy(), rot.copy())
            
            # Direct lookup instead of nested loop (O(1) vs O(m))
            if mocap_data_name in self.bone_name_to_cfg:
                cfg = self.bone_name_to_cfg[mocap_data_name]
                rot_a = R.from_quat(rot, scalar_first=True)
                rot_b = R.from_quat(np.array(cfg.rot_offset), scalar_first=True)
                updated_quat = (rot_a * rot_b).as_quat(scalar_first=True)
                offset_human_data[mocap_data_name] = (
                    offset_human_data[mocap_data_name][0],
                    updated_quat,
                )

                global_pos_offset = R.from_quat(
                    updated_quat, scalar_first=True
                ).apply(cfg.pos_offset)
                offset_human_data[mocap_data_name] = (
                    pos + global_pos_offset,
                    offset_human_data[mocap_data_name][1],
                )

        # Update shared data with lock protection
        with self._data_lock:
            self.mocap_data = offset_human_data

    def _compute_ik_error(self) -> float:
        """Compute total IK error from all tasks."""
        return np.linalg.norm(
            np.concatenate(
                [task.compute_error(self.configuration) for task in self.tasks]
            )
        )

    def mujoco_t(self) -> None:
        """Run Mujoco viewer in a separate thread."""
        try:
            model = self.configuration.model
            data = self.configuration.data
            rate = RateLimiter(frequency=DEFAULT_MUJOCO_VIEWER_FREQUENCY, warn=False)
            with mjv.launch_passive(
                model=model, data=data, show_left_ui=False, show_right_ui=False
            ) as viewer:
                mj.mjv_defaultFreeCamera(model, viewer.cam)

                while viewer.is_running():
                    try:
                        viewer.user_scn.ngeom = 0
                        # Access shared data with lock protection
                        with self._data_lock:
                            mocap_data_copy = self.mocap_data.copy()

                        for _, (pos, rot) in mocap_data_copy.items():
                            draw_frame(
                                pos,
                                R.from_quat(rot, scalar_first=True).as_matrix(),
                                viewer,
                                DEFAULT_FRAME_SIZE,
                                pos_offset=np.array([0.0, 0.0, 0.0]),
                                joint_name=None,
                            )
                        mj.mj_sensorPos(model, data)
                        viewer.sync()
                        rate.sleep()
                    except Exception as e:
                        self.get_logger().error(
                            f"Error in Mujoco viewer loop: {e}", exc_info=True
                        )
                        # Continue running to avoid thread death
                        rate.sleep()
        except Exception as e:
            self.get_logger().error(
                f"Mujoco viewer thread failed: {e}", exc_info=True
            )
