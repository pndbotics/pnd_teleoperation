#!/usr/bin/env python3
"""VR-based Adam robot inverse kinematics node using Mink solver."""

from typing import Callable, Dict

import numpy as np
import rclpy
from adam_mink.adam_mink_vr import AdamMinkVRNode
from adam_mink.constants import (
    DEFAULT_FINGER_POSITION,
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
from shared_utils.shared_utils import JoyAxesIndices, JoyBtnIndices


class AdamMinkVRNodeSG(AdamMinkVRNode):
    """VR SG-based Adam robot IK node with joystick control."""

    def __init__(self) -> None:
        """Initialize the VR SG node."""
        super().__init__("adam_mink_vr_sg")
        self.get_logger().info("Adam Mink VR SG IK node initialized.")

    def init_hand_control(self) -> None:
        """Initialize hand scaling functions for joystick control."""
        self.joy_axes = {
            btn: 0.0
            for btn in [
                JoyAxesIndices.L_trigger,
                JoyAxesIndices.R_trigger,
            ]
        }
        self.joy_btns = {}

        def create_scale_func(joy: Dict, joy_enum, scale: float):
            """Create a scale function for joint control."""
            return lambda: joy.get(joy_enum, 0.0) * scale

        def create_real_scale_func(joy: Dict, joy_enum, scale: float):
            """Create a real scale function for finger control."""
            return lambda: DEFAULT_FINGER_POSITION - joy.get(joy_enum, 0.0) * scale

        self.hands_scale_funcs: Dict[str, Callable[[], float]] = {}

        # Joint mappings
        joint_mappings = [
            (L_GRIP_JOINTS, self.joy_axes, JoyAxesIndices.L_trigger),
            (R_GRIP_JOINTS, self.joy_axes, JoyAxesIndices.R_trigger),
            (L_TRIGGER_JOINTS, self.joy_axes, JoyAxesIndices.L_trigger),
            (R_TRIGGER_JOINTS, self.joy_axes, JoyAxesIndices.R_trigger),
        ]

        for joint_names, joy_dict, joy_enum in joint_mappings:
            for name in joint_names:
                self.hands_scale_funcs[name] = create_scale_func(
                    joy_dict, joy_enum, np.pi / 2
                )
        for name in L_THUMB_ROTATE_JOINTS:
            self.hands_scale_funcs[name] = lambda: np.pi / 2
        for name in R_THUMB_ROTATE_JOINTS:
            self.hands_scale_funcs[name] = lambda: np.pi / 2

        # Finger mappings
        finger_mappings = [
            (L_GRIP_FINGER, self.joy_axes, JoyAxesIndices.L_trigger),
            (L_TRIGGER_FINGER, self.joy_axes, JoyAxesIndices.L_trigger),
            (R_GRIP_FINGER, self.joy_axes, JoyAxesIndices.R_trigger),
            (R_TRIGGER_FINGER, self.joy_axes, JoyAxesIndices.R_trigger),
        ]

        for finger_names, joy_dict, joy_enum in finger_mappings:
            for name in finger_names:
                self.hands_scale_funcs[name] = create_real_scale_func(
                    joy_dict, joy_enum, DEFAULT_FINGER_POSITION
                )
        for name in L_THUMB_ROTATE_FINGER:
            self.hands_scale_funcs[name] = lambda: 0.0
        for name in R_THUMB_ROTATE_FINGER:
            self.hands_scale_funcs[name] = lambda: 0.0
        self.hands_scale_funcs["dof_pos/hand_thumb_1_Left"] = lambda: 1000 - 1000 * (1 - np.exp(-5 * self.joy_axes.get(JoyAxesIndices.L_trigger, 0.0)))/(1 - np.exp(-5))
        self.hands_scale_funcs["dof_pos/hand_thumb_1_Right"] = lambda: 1000 - 1000 * (1 - np.exp(-5 * self.joy_axes.get(JoyAxesIndices.R_trigger, 0.0)))/(1 - np.exp(-5))


def main(args=None) -> None:
    """Main entry point for the VR SG node."""
    rclpy.init(args=args)
    print(f"Starting Adam Mink VR SG IK node... {args}")

    node_sg = AdamMinkVRNodeSG()

    try:
        rclpy.spin(node_sg)
    except KeyboardInterrupt:
        node_sg.get_logger().info("Shutting down node...")
    finally:
        node_sg.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
