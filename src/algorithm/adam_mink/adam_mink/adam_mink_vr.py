#!/usr/bin/env python3
"""VR-based Adam robot inverse kinematics node using Mink solver."""

import rclpy
from typing import Callable, Dict, List, Tuple

import numpy as np
from adam_mink.adam_mink_base import (
    AdamMinkBase,
    DEFAULT_FINGER_POSITION,
    DEFAULT_TIMER_PERIOD,
    ROOT_POSE_NUM,
)
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
from sensor_msgs.msg import Joy
from shared_utils.shared_utils import JoyAxesIndices, JoyBtnIndices

# Constants
DEFAULT_JOYSTICK_QUEUE_SIZE = 10
DEFAULT_ZERO_VELOCITY = 0.5  # rad/s, velocity for smooth zeroing


class AdamMinkVRNode(AdamMinkBase):
    """VR-based Adam robot IK node with joystick control."""

    def __init__(self) -> None:
        """Initialize the VR node."""
        super().__init__("adam_mink_vr")
        self.calibrated = False

        # Initialize zeroing state
        self.current_joint_positions = None  # Will be initialized on first call
        self.zeroing_initialized = False

        self.sim_joint_num = len(self.robot_motor_names)

        self.sub_joy = self.create_subscription(
            Joy, "vr/joy", self.joy_callback, DEFAULT_JOYSTICK_QUEUE_SIZE
        )
        self.init_hand_control()
        self.get_logger().info("Adam Mink VR IK node initialized.")

    def cb_transform(self):
        if not self.calibrated:
            self.adam_zero_callback()
            return
        else:
            return super().cb_transform()

    def get_bone_frames(self) -> List[str]:
        """Get the list of bone frame names to track for VR."""
        return ["LeftHand", "RightHand", "Head"]

    def _initialize_mocap_data(self) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        mock_mocap_data = {
            "root": (np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])),
            "Spine2": (np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])),
            "LeftArm": (np.array([0.0, 0.3, 0.0]), np.array([1.0, 0.1, 0.0, 0.0])),
            "RightArm": (np.array([0.0, -0.3, 0.0]), np.array([1.0, -0.1, 0.0, 0.0])),
        }
        mocap_data = super()._initialize_mocap_data()
        mocap_data.update(mock_mocap_data)
        return mocap_data
        
    def init_hand_control(self) -> None:
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

    def update_joint_states(self) -> None:
        """Update joint states from joystick input."""
        for name, func in self.hands_scale_funcs.items():
            idx = self.all_joint.get(name)
            if idx is not None:
                self.joint_state_msg.position[ROOT_POSE_NUM + idx] = func()
            else:
                self.get_logger().warn(f"Motor name {name} not found in all_joint")

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
            try:
                if (
                    hasattr(self.configuration, "data")
                    and self.configuration.data.qpos is not None
                ):
                    # Skip first 7 (root pos/rot) and use joint positions
                    config_positions = self.configuration.data.qpos[
                        ROOT_POSE_NUM : ROOT_POSE_NUM + self.sim_joint_num
                    ]
                    if len(config_positions) == self.sim_joint_num:
                        self.current_joint_positions = np.array(config_positions)
                    else:
                        self.current_joint_positions = np.zeros(self.sim_joint_num)
                else:
                    self.current_joint_positions = np.zeros(self.sim_joint_num)
            except Exception as e:
                self.get_logger().error(f"Error initializing zero positions: {e}")
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
            list([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            + list(self.current_joint_positions)
            + [DEFAULT_FINGER_POSITION] * self.finger_joint_num
        )
        self.joint_state_pub.publish(self.joint_state_msg)

        # Update configuration data (first 7 are root pos/rot, then joint positions)
        try:
            root_pos_rot = [0.0] * ROOT_POSE_NUM
            self.configuration.data.qpos = root_pos_rot + list(
                self.current_joint_positions
            )
        except Exception as e:
            self.get_logger().error(f"Error updating configuration data: {e}")


def main(args=None) -> None:
    """Main entry point for the VR node."""
    rclpy.init(args=args)
    print(f"Starting Adam Mink VR IK node... {args}")

    node = AdamMinkVRNode()

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
