#!/usr/bin/env python3
"""Pico Tracker 3-based Adam robot inverse kinematics node using Mink solver."""

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from adam_mink.adam_mink_base import AdamMinkBase
from adam_mink.constants import (
    DEFAULT_FINGER_POSITION,
    DEFAULT_TIMER_PERIOD,
    ROOT_POSE_NUM,
)

MocapData = dict[str, tuple[np.ndarray, np.ndarray]]

DEFAULT_JOYSTICK_QUEUE_SIZE = 10
DEFAULT_ZERO_VELOCITY = 0.5  # rad/s, velocity for smooth zeroing


class AdamMinkPicoWbNode(AdamMinkBase):
    """Pico Wb-based Adam robot IK node with joystick control."""

    def __init__(self, node_name: str = "adam_mink_picowb") -> None:
        """Initialize the Pico Wb node."""
        super().__init__(node_name)
        self.calibrated = True

        # Initialize zeroing state
        self.current_joint_positions = None  # Will be initialized on first call
        self.zeroing_initialized = False

        self.sim_joint_num = len(self.robot_motor_names)

        self.get_logger().info("Adam Mink Pico Wb IK node initialized.")

    def transform_callback(self) -> None:
        """Override transform callback to handle calibration state."""
        return super().transform_callback()

    def get_bone_frames(self) -> list[str]:
        """Get the list of bone frame names to track for Pico."""
        return [
            "pelvis",
            "neck",
            "l-shoulder",
            "r-shoulder",
            "spine-01",
            "l-elbow",
            "l-hand",
            "r-elbow",
            "r-hand",
        ]

    def _initialize_mocap_data(self) -> MocapData:
        mock_mocap_data = {
            "root": (np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0])),
        }
        mocap_data = super()._initialize_mocap_data()
        mocap_data.update(mock_mocap_data)
        return mocap_data

    def adam_zero_callback(self) -> None:
        """Smoothly reset all joints to zero position."""
        if not self.zeroing_initialized:
            self._initialize_zero_positions()

        self._update_zero_positions()
        self._publish_zero_joint_states()
        self._update_zero_configuration()

    def _initialize_zero_positions(self) -> None:
        """Initialize current joint positions from configuration for zeroing."""
        try:
            if hasattr(self.configuration, "data") and self.configuration.data.qpos is not None:
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

    def _update_zero_positions(self) -> None:
        """Update joint positions smoothly towards zero with velocity limit."""
        dt = DEFAULT_TIMER_PERIOD
        max_velocity = DEFAULT_ZERO_VELOCITY
        max_step = max_velocity * dt

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

    def _publish_zero_joint_states(self) -> None:
        """Publish joint states with zero positions."""
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = (
            [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0]
            + list(self.current_joint_positions)
            + [DEFAULT_FINGER_POSITION] * self.finger_joint_num
        )
        self.joint_state_pub.publish(self.joint_state_msg)

    def _update_zero_configuration(self) -> None:
        """Update configuration data with zero positions."""
        try:
            root_pos_rot = [0.0] * ROOT_POSE_NUM
            self.configuration.data.qpos = root_pos_rot + list(self.current_joint_positions)
        except Exception as e:
            self.get_logger().error(f"Error updating configuration data: {e}")


def main(args=None) -> None:
    """Main entry point for the Pico Wb node."""
    rclpy.init(args=args)
    print(f"Starting Adam Mink Pico Wb IK node... {args}")

    node = AdamMinkPicoWbNode()

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
