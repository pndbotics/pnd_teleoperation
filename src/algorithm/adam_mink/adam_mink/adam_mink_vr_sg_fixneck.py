#!/usr/bin/env python3
"""VR SG-based Adam robot inverse kinematics node with neck fix."""

import numpy as np
import rclpy

from adam_mink.adam_mink_vr_sg import AdamMinkVRNodeSG


class AdamMinkVRNodeSGFixNeck(AdamMinkVRNodeSG):
    """VR SG-based Adam robot IK node with joystick control."""

    def __init__(self) -> None:
        """Initialize the VR SG node."""
        super().__init__("adam_mink_vr_sg_fixneck")
        self.get_logger().info("Adam Mink VR SG FixNeck IK node initialized.")

    def update_joint_states(self) -> None:
        super().update_joint_states()
        # Fix neck joints to zero
        self.joint_state_msg.position[49] = np.pi / 4 + 0.05
        self.joint_state_msg.position[48] = 0.0


def main(args=None) -> None:
    """Main entry point for the VR SG node."""
    rclpy.init(args=args)
    print(f"Starting Adam Mink VR SG FixNeck IK node... {args}")

    node_sg = AdamMinkVRNodeSGFixNeck()

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
