#!/usr/bin/env python3
"""ROS2 node for Adam robot inverse kinematics using Mink solver."""

from threading import Lock, Thread
from typing import Callable, Dict, Tuple

import mink
import mujoco as mj
import mujoco.viewer as mjv
import numpy as np
import rclpy
import tf2_ros
import yaml
from adam_mink.constants import (
    ALL_FINGER,
    L_GRIP_FINGER,
    L_GRIP_JOINTS,
    L_THUMB_ROTATE_FINGER,
    L_THUMB_ROTATE_JOINTS,
    L_TRIGGER_FINGER,
    L_TRIGGER_JOINTS,
    R_GRIP_FINGER,
    R_GRIP_JOINTS,
    R_THUMB_ROTATE_FINGER,
    R_THUMB_ROTATE_JOINTS,
    R_TRIGGER_FINGER,
    R_TRIGGER_JOINTS,
)
from adam_mink.utils import draw_frame
from loop_rate_limiters import RateLimiter
from pydantic import BaseModel
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState, Joy
from shared_utils.shared_utils import JoyAxesIndices, JoyBtnIndices

# Constants
DEFAULT_TIMER_PERIOD = 0.01  # 100 Hz
DEFAULT_JOYSTICK_QUEUE_SIZE = 10
DEFAULT_JOINT_STATE_QUEUE_SIZE = 10
DEFAULT_MUJOCO_VIEWER_FREQUENCY = 200.0
DEFAULT_IK_SOLVER = "daqp"
DEFAULT_IK_DAMPING = 3e-1
DEFAULT_IK_ITER_MAX = 3
DEFAULT_IK_ERROR_THRESHOLD = 0.001
DEFAULT_IK_ITER_WARN_THRESHOLD = 0.8
DEFAULT_FINGER_POSITION = 1000.0
DEFAULT_JOINT_POSITION_INVALID = 1000.0
DEFAULT_FRAME_SIZE = 0.1
DEFAULT_ARROW_WIDTH = 0.005
DEFAULT_ARROW_SIZE = [0.01, 0.01, 0.01]
DEFAULT_ZERO_VELOCITY = 0.5  # rad/s, velocity for smooth zeroing


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


class TF2ListenerNode(Node):
    """ROS2 node for Adam robot IK using Mink solver with TF2 transforms."""

    def __init__(self) -> None:
        """Initialize the TF2 listener node."""
        super().__init__("adam_mink")
        self.declare_parameter("adam_model_path", "")
        self.declare_parameter("adam_mink_cfg", "")
        self.declare_parameter("mujoco_sim", False)
        adam_mink_cfg_path = self.get_parameter("adam_mink_cfg").value
        adam_model_path = self.get_parameter("adam_model_path").value
        mujoco_sim = self.get_parameter("mujoco_sim").value

        # Validate required parameters
        if not adam_model_path:
            raise ValueError(
                "Required parameter 'adam_model_path' is not set or is empty"
            )
        if not adam_mink_cfg_path:
            raise ValueError(
                "Required parameter 'adam_mink_cfg' is not set or is empty"
            )

        self.get_logger().info(f"adam_mink_cfg_path: {adam_mink_cfg_path}")
        self.get_logger().info(f"adam_model_path: {adam_model_path}")

        self.calibrated = False

        # Thread synchronization lock for shared data
        self._data_lock = Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub_joy = self.create_subscription(
            Joy, "vr/joy", self.joy_callback, DEFAULT_JOYSTICK_QUEUE_SIZE
        )
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", DEFAULT_JOINT_STATE_QUEUE_SIZE
        )
        self.timer = self.create_timer(DEFAULT_TIMER_PERIOD, self.check_transform)
        self.bone_frames = ["LeftHand", "RightHand", "Head"]
        self.mocap_data = {
            "root": (np.array([0, 0, 0.0]), np.array([1, 0, 0, 0])),
            "Spine2": (np.array([0, 0, 0.0]), np.array([1, 0, 0, 0])),
            "LeftArm": (np.array([0, 0.3, 0.0]), np.array([1, 0.1, 0, 0])),
            "RightArm": (np.array([0, -0.3, 0.0]), np.array([1, -0.1, 0, 0])),
        }

        self.model = mj.MjModel.from_xml_path(adam_model_path)
        self.robot_motor_names = {}
        for i in range(self.model.nu):  # 'nu' is the number of actuators (motors)
            motor_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i)
            self.robot_motor_names[f"dof_pos/{motor_name}"] = i
            self.get_logger().info(f"Actuator {i}: {motor_name}")
        self.sim_joint_num = len(self.robot_motor_names)
        self.finger_joint_num = len(ALL_FINGER)
        self.all_joint = {
            name: i
            for i, name in enumerate(
                list(self.robot_motor_names.keys()) + list(ALL_FINGER)
            )
        }
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = list(self.all_joint.keys())
        self.joint_state_msg.position = [DEFAULT_JOINT_POSITION_INVALID] * len(
            self.all_joint
        )

        # Initialize zeroing state
        self.current_joint_positions = None  # Will be initialized on first call
        self.zeroing_initialized = False

        self.init_hands_scale_funcs()

        self.configuration = mink.Configuration(self.model)
        with open(adam_mink_cfg_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        self.adam_mink_cfg = AdamMinkConfig.model_validate(data)
        self.get_logger().info(f"Loaded config: {self.adam_mink_cfg}")

        self.tasks = self._create_ik_tasks()
        self.limits = self._create_ik_limits()

        # Start Mujoco viewer thread
        if mujoco_sim:
            mj_t = Thread(target=self.mujoco_t, daemon=True)
            mj_t.start()

        self.get_logger().info("TF2 listener node started")

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

    def _create_ik_limits(self) -> list:
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
        limits = [mink.ConfigurationLimit(self.model)]
        limits.extend(collision_avoidance_limit)

        max_velocities = {
            "shoulderPitch_Left": 3 * np.pi,
            "shoulderPitch_Right": 3 * np.pi,
            "shoulderRoll_Left": 3 * np.pi,
            "shoulderRoll_Right": 3 * np.pi,
        }
        velocity_limit = mink.VelocityLimit(self.model, max_velocities)
        limits.append(velocity_limit)
        return limits

    def init_hands_scale_funcs(self) -> None:
        """Initialize hand scaling functions for joystick control."""
        self.joy_axes = {
            btn: 0.0
            for btn in [
                JoyAxesIndices.L_trigger,
                JoyAxesIndices.L_grip,
                JoyAxesIndices.R_trigger,
                JoyAxesIndices.R_grip,
            ]
        }
        self.joy_btns = {
            btn: 0 for btn in [JoyBtnIndices.L_X_touch, JoyBtnIndices.R_A_touch]
        }

        def create_scale_func(joy: Dict, joy_enum, scale: float):
            """Create a scale function for joint control."""
            return lambda: joy.get(joy_enum, 0.0) * scale

        def create_real_scale_func(joy: Dict, joy_enum, scale: float):
            """Create a real scale function for finger control."""
            return lambda: DEFAULT_FINGER_POSITION - joy.get(joy_enum, 0.0) * scale

        self.hands_scale_funcs: Dict[str, Callable[[], float]] = {}

        # Joint mappings
        joint_mappings = [
            (L_GRIP_JOINTS, self.joy_axes, JoyAxesIndices.L_grip),
            (R_GRIP_JOINTS, self.joy_axes, JoyAxesIndices.R_grip),
            (L_TRIGGER_JOINTS, self.joy_axes, JoyAxesIndices.L_trigger),
            (R_TRIGGER_JOINTS, self.joy_axes, JoyAxesIndices.R_trigger),
            (L_THUMB_ROTATE_JOINTS, self.joy_btns, JoyBtnIndices.L_X_touch),
            (R_THUMB_ROTATE_JOINTS, self.joy_btns, JoyBtnIndices.R_A_touch),
        ]

        for joint_names, joy_dict, joy_enum in joint_mappings:
            for name in joint_names:
                self.hands_scale_funcs[name] = create_scale_func(
                    joy_dict, joy_enum, np.pi / 2
                )

        # Finger mappings
        finger_mappings = [
            (L_GRIP_FINGER, self.joy_axes, JoyAxesIndices.L_grip),
            (L_TRIGGER_FINGER, self.joy_axes, JoyAxesIndices.L_trigger),
            (L_THUMB_ROTATE_FINGER, self.joy_btns, JoyBtnIndices.L_X_touch),
            (R_GRIP_FINGER, self.joy_axes, JoyAxesIndices.R_grip),
            (R_TRIGGER_FINGER, self.joy_axes, JoyAxesIndices.R_trigger),
            (R_THUMB_ROTATE_FINGER, self.joy_btns, JoyBtnIndices.R_A_touch),
        ]

        for finger_names, joy_dict, joy_enum in finger_mappings:
            for name in finger_names:
                self.hands_scale_funcs[name] = create_real_scale_func(
                    joy_dict, joy_enum, DEFAULT_FINGER_POSITION
                )

    def check_transform(self) -> None:
        """Check TF transforms and update IK solution."""

        if not self._update_mocap_data():
            return
        if self.calibrated:
            self.offset_mocap_data()
            self._update_ik_targets()
            self._solve_ik()
            self._publish_joint_states()
        else:
            self.adam_zero_callback()

    def _update_mocap_data(self) -> bool:
        """Update mocap data from TF transforms."""
        mocap_data_update = {}
        for bone in self.bone_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "world", bone, rclpy.time.Time()
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
                self.get_logger().warn(
                    f"Transform connectivity exception: world -> {bone}"
                )
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
        # Access shared data with lock protection
        with self._data_lock:
            mocap_data_copy = self.mocap_data.copy()

        for task, cfg in zip(self.tasks, self.adam_mink_cfg.ik_cfg):
            pos, rot = mocap_data_copy[cfg.bone_name]
            task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))

    def _solve_ik(self) -> None:
        """Solve inverse kinematics iteratively."""
        curr_error = self._compute_ik_error()
        dt = self.configuration.model.opt.timestep
        num_iter = 0

        while num_iter < DEFAULT_IK_ITER_MAX:
            vel = mink.solve_ik(
                configuration=self.configuration,
                tasks=self.tasks,
                dt=dt,
                solver=DEFAULT_IK_SOLVER,
                damping=DEFAULT_IK_DAMPING,
                limits=self.limits,
            )
            self.configuration.integrate_inplace(vel, dt)
            next_error = self._compute_ik_error()

            if curr_error - next_error <= DEFAULT_IK_ERROR_THRESHOLD:
                break

            curr_error = next_error
            num_iter += 1

        if num_iter > DEFAULT_IK_ITER_MAX * DEFAULT_IK_ITER_WARN_THRESHOLD:
            self.get_logger().debug(
                f"IK iteration count is high: {num_iter}, error: {next_error:.6f}"
            )

    def _publish_joint_states(self) -> None:
        """Publish joint states to ROS topic."""
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        # Assuming first 7 are root pos/rot
        self.joint_state_msg.position = (
            list(self.configuration.data.qpos[7:])
            + [DEFAULT_FINGER_POSITION] * self.finger_joint_num
        )

        for name, func in self.hands_scale_funcs.items():
            idx = self.all_joint.get(name)
            if idx is not None:
                self.joint_state_msg.position[idx] = func()
            else:
                self.get_logger().warning(f"Motor name {name} not found in all_joint")

        self.joint_state_pub.publish(self.joint_state_msg)

    def offset_mocap_data(self) -> None:
        """Apply position and rotation offsets to mocap data."""
        # Access shared data with lock protection
        with self._data_lock:
            mocap_data_copy = self.mocap_data.copy()

        offset_human_data: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for mocap_data_name, (pos, rot) in mocap_data_copy.items():
            offset_human_data[mocap_data_name] = (pos.copy(), rot.copy())
            for cfg in self.adam_mink_cfg.ik_cfg:
                if mocap_data_name == cfg.bone_name:
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

    def joy_callback(self, msg: Joy) -> None:
        """Handle joystick input messages."""
        if msg.buttons[JoyBtnIndices.R_A] == 1 and not self.calibrated:
            self.calibrated = True
            self.get_logger().info("Calibration activated")
        if msg.buttons[JoyBtnIndices.R_B] == 1 and self.calibrated:
            self.calibrated = False
            self.get_logger().info("Calibration deactivated")
            # Reset zeroing state when entering zero mode
            self.zeroing_initialized = False

        for btn in self.joy_btns.keys():
            self.joy_btns[btn] = msg.buttons[btn.value]
        for btn in self.joy_axes.keys():
            self.joy_axes[btn] = msg.axes[btn.value]

    def adam_zero_callback(self) -> None:
        """Smoothly reset all joints to zero position."""
        dt = DEFAULT_TIMER_PERIOD
        max_velocity = DEFAULT_ZERO_VELOCITY
        max_step = max_velocity * dt

        # Initialize current positions from configuration on first call
        if not self.zeroing_initialized:
            if (
                hasattr(self.configuration, "data")
                and self.configuration.data.qpos is not None
            ):
                # Skip first 7 (root pos/rot) and use joint positions
                config_positions = self.configuration.data.qpos[
                    7 : 7 + self.sim_joint_num
                ]
                if len(config_positions) == self.sim_joint_num:
                    self.current_joint_positions = np.array(config_positions)
                else:
                    self.current_joint_positions = np.zeros(self.sim_joint_num)
            else:
                self.current_joint_positions = np.zeros(self.sim_joint_num)
            self.zeroing_initialized = True

        # Smoothly move towards zero
        target_positions = np.zeros(self.sim_joint_num)
        position_diffs = target_positions - self.current_joint_positions
        position_norms = np.abs(position_diffs)

        # Move each joint towards zero with velocity limit
        for i in range(self.sim_joint_num):
            if position_norms[i] > max_step:
                # Move by max_step towards zero
                direction = -1.0 if position_diffs[i] < 0 else 1.0
                self.current_joint_positions[i] += direction * max_step
            else:
                # Close enough, set to zero
                self.current_joint_positions[i] = 0.0

        # Update joint state message
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = (
            list(self.current_joint_positions)
            + [DEFAULT_FINGER_POSITION] * self.finger_joint_num
        )
        self.joint_state_pub.publish(self.joint_state_msg)

        # Update configuration data (first 7 are root pos/rot, then joint positions)
        root_pos_rot = [0.0] * 7
        self.configuration.data.qpos = root_pos_rot + list(self.current_joint_positions)

    def mujoco_t(self) -> None:
        """Run Mujoco viewer in a separate thread."""
        model = self.configuration.model
        data = self.configuration.data
        rate = RateLimiter(frequency=DEFAULT_MUJOCO_VIEWER_FREQUENCY, warn=False)
        with mjv.launch_passive(
            model=model, data=data, show_left_ui=False, show_right_ui=False
        ) as viewer:
            mj.mjv_defaultFreeCamera(model, viewer.cam)

            while viewer.is_running():
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
                mj.mj_fwdPosition(model, data)
                mj.mj_sensorPos(model, data)
                viewer.sync()
                rate.sleep()


def main(args=None) -> None:
    """Main entry point for the node."""
    rclpy.init(args=args)

    node = TF2ListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
