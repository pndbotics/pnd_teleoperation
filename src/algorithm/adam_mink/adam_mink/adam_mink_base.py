#!/usr/bin/env python3
"""Base class for Adam robot inverse kinematics using Mink solver."""

from abc import ABC, abstractmethod
from threading import Lock, Thread
from typing import Dict, List, Tuple, Union

import mink
import mujoco as mj
import mujoco.viewer as mjv
import numpy as np

# Type aliases for better readability
MocapData = Dict[str, Tuple[np.ndarray, np.ndarray]]  # bone_name -> (position, rotation)
import rclpy
import tf2_ros
import yaml
import array
from adam_mink.utils import draw_frame, quat_mul_single, quat_rotate_vector
from adam_mink.constants import (
    ALL_FINGER,
    DEFAULT_TIMER_PERIOD,
    DEFAULT_JOINT_STATE_QUEUE_SIZE,
    DEFAULT_MUJOCO_VIEWER_FREQUENCY,
    DEFAULT_IK_SOLVER,
    DEFAULT_IK_DAMPING,
    DEFAULT_IK_ITER_MAX,
    DEFAULT_IK_ERROR_THRESHOLD,
    DEFAULT_IK_ITER_WARN_THRESHOLD,
    DEFAULT_FINGER_POSITION,
    DEFAULT_FRAME_SIZE,
    ROOT_POSE_NUM,
)
from loop_rate_limiters import RateLimiter
from pydantic import BaseModel
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import time


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
        
        # Load and validate parameters
        adam_model_path, adam_mink_cfg_path, mujoco_sim = self._load_parameters()
        
        # Initialize ROS2 components
        self._initialize_ros_components()
        
        # Get bone frames from subclass (bones to track from TF)
        self.bone_frames = self.get_bone_frames()
        
        # Load MuJoCo model
        self._load_model(adam_model_path)
        
        # Load and validate configuration
        self._load_config(adam_mink_cfg_path)
        
        # Initialize joint and motor mappings
        self._initialize_joint_mappings()
        
        # Initialize IK solver
        self._initialize_ik_solver()
        
        # Start background threads
        self._start_background_threads(mujoco_sim)
        
        self.get_logger().info(f"{node_name} node started")

    def _load_parameters(self) -> Tuple[str, str, bool]:
        """Load and validate ROS2 parameters.
        
        Returns:
            Tuple of (adam_model_path, adam_mink_cfg_path, mujoco_sim).
        """
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
        
        return adam_model_path, adam_mink_cfg_path, mujoco_sim

    def _initialize_ros_components(self) -> None:
        """Initialize ROS2 publishers, subscribers, and TF components."""
        self.base_frame = "world"
        self.calibrated = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", DEFAULT_JOINT_STATE_QUEUE_SIZE
        )
        self.timer = self.create_timer(DEFAULT_TIMER_PERIOD, self.transform_callback)

    def _load_model(self, adam_model_path: str) -> None:
        """Load MuJoCo model from file.
        
        Args:
            adam_model_path: Path to the MuJoCo XML model file.
        """
        try:
            self.model = mj.MjModel.from_xml_path(adam_model_path)
        except Exception as e:
            self.get_logger().fatal(
                f"Failed to load MuJoCo model from {adam_model_path}: {e}"
            )
            raise
        
        # Initialize configuration
        self.configuration = mink.Configuration(self.model)

    def _load_config(self, adam_mink_cfg_path: str) -> None:
        """Load and validate configuration file.
        
        Args:
            adam_mink_cfg_path: Path to the YAML configuration file.
        """
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
        
        # Validate config structure
        try:
            self.adam_mink_cfg = AdamMinkConfig.model_validate(data)
        except Exception as e:
            self.get_logger().fatal(f"Failed to validate config: {e}")
            raise
        
        # Validate config completeness
        if not self.adam_mink_cfg.ik_cfg:
            raise ValueError("IK configuration is empty")
        
        # Create bone_name -> cfg mapping to avoid nested loop
        self.bone_name_to_cfg = {
            cfg.bone_name: cfg for cfg in self.adam_mink_cfg.ik_cfg
        }
        
        self._rot_offset_quats = {}
        self._pos_offsets = {}
        for cfg in self.adam_mink_cfg.ik_cfg:
            if cfg.rot_offset:
                self._rot_offset_quats[cfg.bone_name] = np.array(cfg.rot_offset, dtype=np.float64)
            if cfg.pos_offset:
                self._pos_offsets[cfg.bone_name] = np.array(cfg.pos_offset, dtype=np.float64)

        # Warn about missing bone frames in config
        for bone in self.bone_frames:
            if not any(cfg.bone_name == bone for cfg in self.adam_mink_cfg.ik_cfg):
                self.get_logger().warning(
                    f"Bone frame '{bone}' not found in IK config"
                )
        
        self.get_logger().info(f"Loaded config: {self.adam_mink_cfg}")

        # Initialize mocap_data with all bones from config (not just tracked ones)
        self.mocap_data = self._initialize_mocap_data()
        self.mocap_data_adjusted = {}
        self._data_lock = Lock()

    def _initialize_joint_mappings(self) -> None:
        """Initialize joint and motor name mappings."""
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
        self._qpos_size = self.configuration.data.qpos.size

    def _initialize_ik_solver(self) -> None:
        """Initialize IK tasks and limits."""
        self.tasks = self._create_ik_tasks()
        self.limits = self._create_ik_limits()

    def _start_background_threads(self, mujoco_sim: bool) -> None:
        """Start background threads for IK solving and visualization.
        
        Args:
            mujoco_sim: Whether to start MuJoCo viewer thread.
        """
        self.ik_thread = Thread(target=self.ik_thread_loop, daemon=True)
        self.ik_thread.start()

        # Start Mujoco viewer thread
        if mujoco_sim:
            mj_t = Thread(target=self.mujoco_t, daemon=True)
            mj_t.start()

    def ik_thread_loop(self) -> None:
        """Target function for the IK thread."""
        self.get_logger().info("IK thread loop started")
        while rclpy.ok():
            if not self.calibrated:
                time.sleep(DEFAULT_TIMER_PERIOD)
                continue
            
            start_time = time.time()
            with self._data_lock:
                mocap_data_copy = self.mocap_data.copy()
            if self.adam_mink_cfg.human_scale_table:
                self.scale_mocap_data(mocap_data_copy)
            scale_time = time.time()
            if scale_time - start_time > 0.001:
                self.get_logger().warning(f"Scale mocap data took {scale_time - start_time} seconds")
            self.offset_mocap_data(mocap_data_copy)
            offset_time = time.time()
            if offset_time - scale_time > 0.001:
                self.get_logger().warning(f"Offset mocap data took {offset_time - scale_time} seconds")
            self.mocap_data_adjusted = mocap_data_copy
            self._update_ik_targets()
            update_ik_targets_time = time.time()
            if update_ik_targets_time - offset_time > 0.001:
                self.get_logger().warning(f"Update IK targets took {update_ik_targets_time - offset_time} seconds")
            self._solve_ik()
            solve_ik_time = time.time()
            if solve_ik_time - update_ik_targets_time > 0.02:
                self.get_logger().warning(f"Solve IK took {solve_ik_time - update_ik_targets_time} seconds")
            self._publish_joint_states()
            publish_joint_states_time = time.time()
            if publish_joint_states_time - solve_ik_time > 0.001:
                self.get_logger().warning(f"Publish joint states took {publish_joint_states_time - solve_ik_time} seconds")
            end_time = time.time()
            # if end_time - start_time > 0.025:
            #     self.get_logger().warning(f"IK thread loop took {end_time - start_time} seconds, which is longer than 25ms")
            
        self.get_logger().info("IK thread loop ended")


    @abstractmethod
    def get_bone_frames(self) -> list[str]:
        """Get the list of bone frame names to track.

        Returns:
            List of bone frame names.
        """
        pass

    def _initialize_mocap_data(self) -> MocapData:
        """Initialize mocap data dictionary with all bones from config.
        
        Returns:
            Dictionary mapping bone names to (position, rotation) tuples.
        """
        # Initialize all bones from config (some may not be tracked from TF)
        unique_bones = {cfg.bone_name for cfg in self.adam_mink_cfg.ik_cfg}
        default_pos = np.array([0.0, 0.0, 0.0])
        default_rot = np.array([1.0, 0.0, 0.0, 0.0])
        return {
            bone_name: (default_pos.copy(), default_rot.copy())
            for bone_name in unique_bones
        }

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

    def transform_callback(self) -> None:
        """Callback for TF transform timer. Check TF transforms and update mocap data."""
        if not self._update_mocap_data():
            return
        
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
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warning(
                f"Transform connectivity exception: {self.base_frame} -> {bone}: {e}"
            )
            return False
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warning(
                f"Transform extrapolation exception: {self.base_frame} -> {bone}: {e}"
            )
            return False

        # Update shared data with lock protection
        with self._data_lock:
            self.mocap_data.update(mocap_data_update)
        return True

    def _update_ik_targets(self) -> bool:
        """Update IK task targets from mocap data."""
        for task, cfg in zip(self.tasks, self.adam_mink_cfg.ik_cfg):
            mocap_item = self.mocap_data_adjusted.get(cfg.bone_name)
            if mocap_item is not None:
                pos, rot = mocap_item
                task.set_target(
                    mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos)
                )
            else:
                self.get_logger().warning(
                    f"Bone '{cfg.bone_name}' not found in mocap_data"
                )
        return True

    def _solve_ik(self) -> None:
        """Solve inverse kinematics iteratively."""
        dt = self.configuration.model.opt.timestep
        num_iter = 0

        while num_iter < self.ik_iter_max:
            vel = mink.solve_ik(
                configuration=self.configuration,
                tasks=self.tasks,
                dt=dt,
                solver=self.ik_solver,
                damping=self.ik_damping,
                limits=self.limits,
            )

            self.configuration.integrate_inplace(vel, dt)
            num_iter += 1

    def _publish_joint_states(self) -> None:
        """Publish joint states to ROS topic."""
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position[:self._qpos_size] = array.array('d', self.configuration.data.qpos.tolist())
        self.update_joint_states()
        self.joint_state_pub.publish(self.joint_state_msg)

    def update_joint_states(self) -> None:
        """Update joint states. Override in subclass if needed."""
        pass

    def scale_mocap_data(self, data: MocapData) -> None:
        """Scale mocap data based on human scale table.
        
        Args:
            data: Mocap data dictionary to scale in-place.
        """
        human_data_local = {}
        root_pos, root_quat = data["Hips"]

        # scale root
        scaled_root_pos = self.adam_mink_cfg.human_scale_table["Hips"] * root_pos

        # scale other body parts in local frame
        for body_name in data.keys():
            if body_name not in self.adam_mink_cfg.human_scale_table.keys():
                continue
            if body_name == "Hips":
                continue
            else:
                # transform to local frame (only position)
                human_data_local[body_name] = (
                    data[body_name][0] - root_pos
                ) * self.adam_mink_cfg.human_scale_table[body_name]

        # transform the human data back to the global frame
        human_data_global = {"Hips": (scaled_root_pos, root_quat)}
        for body_name in human_data_local.keys():
            human_data_global[body_name] = (
                human_data_local[body_name] + scaled_root_pos,
                data[body_name][1],
            )

        data.update(human_data_global)

    def offset_mocap_data(self, data: MocapData) -> None:
        """Apply position and rotation offsets to mocap data.
        
        Args:
            data: Mocap data dictionary to apply offsets to in-place.
        """
        bone_name_to_cfg = self.bone_name_to_cfg
        rot_offset_quats = self._rot_offset_quats
        pos_offsets = self._pos_offsets
        
        for mocap_data_name, (pos, rot) in data.items():
            if mocap_data_name in bone_name_to_cfg:
                rot_b = rot_offset_quats.get(mocap_data_name)
                pos_offset = pos_offsets.get(mocap_data_name)
                
                if rot_b is None and pos_offset is None:
                    continue
                
                if rot_b is not None:
                    rot_combined = quat_mul_single(rot, rot_b)
                else:
                    rot_combined = rot
                
                if pos_offset is not None:
                    global_pos_offset = quat_rotate_vector(rot_combined, pos_offset)
                    pos = pos + global_pos_offset
                
                data[mocap_data_name] = (pos, rot_combined)

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
                        for _, (pos, rot) in self.mocap_data_adjusted.items():
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
                        rate.sleep()
        except Exception as e:
            self.get_logger().error(
                f"Mujoco viewer thread failed: {e}", exc_info=True
            )
