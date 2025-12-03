#!/usr/bin/env python3
"""ROS2 node for Adam robot inverse kinematics using Mink solver (VR version)."""

import rclpy
from adam_mink.adam_mink_vr import AdamMinkVRNode


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
