import asyncio
import os
from dataclasses import dataclass, field
from pathlib import Path
from threading import Thread
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Transform, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from shared_utils.shared_utils import JoyAxesIndices, JoyBtnIndices
from tf2_ros import TransformBroadcaster
from webvr_mocap.utils import count_elements
from webvr_mocap.vr.vr_monitor import VRMonitor

# Constants
DEFAULT_PUBLISH_RATE = 0.01  # 100 Hz
DEFAULT_ROBOT_SCALE = 1.0
DEFAULT_ROBOT_ARM_LENGTH = 0.55  # T-pose arm length in meters
DEFAULT_HEAD_DISTANCE = 0.7  # Default head position distance
DEFAULT_HAND_DISTANCE = 0.5  # Default hand position distance
GOAL_BONES = ["Head", "LeftHand", "RightHand"]


@dataclass
class PositionOffset:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class RotationOffset:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class TransformOffset:
    position: PositionOffset = field(default_factory=PositionOffset)
    rotation: RotationOffset = field(default_factory=RotationOffset)


class WebVRMocap(Node):
    """WebVR Mocap Node for processing VR controller data and publishing ROS transforms."""

    def __init__(self):
        super().__init__("webvr_mocap")
        self.goal_bones = GOAL_BONES
        self._data: List[TransformStamped] = self._initialize_transform_data()
        self._pub = TransformBroadcaster(self)
        self.joy_pub = self.create_publisher(Joy, "vr/joy", 10)

        # Calibration parameters
        self.calibrated = False
        self.robot_scale = DEFAULT_ROBOT_SCALE
        self.robot_arm_length = DEFAULT_ROBOT_ARM_LENGTH
        self.transform_offsets = {bone: TransformOffset() for bone in self.goal_bones}
        self.t_pos_dist = {
            "Head": DEFAULT_HEAD_DISTANCE,
            "LeftHand": DEFAULT_HAND_DISTANCE,
            "RightHand": DEFAULT_HAND_DISTANCE,
        }

        # Initialize VR monitor
        self.vr_monitor: Optional[VRMonitor] = None
        self.start_vr_monitor()

        # Start data publishing timer
        self.create_timer(DEFAULT_PUBLISH_RATE, self.receive_data)

        self.get_logger().info("VR Mocap Node Initialized")

    def _initialize_transform_data(self) -> List[TransformStamped]:
        """Initialize transform data structures for each bone."""
        data = []
        for bone in self.goal_bones:
            t = TransformStamped()
            t.header.frame_id = "world"
            t.child_frame_id = bone
            t.transform = Transform()
            data.append(t)
        return data

    def get_workspace_root(self) -> str:
        """Get workspace root directory.

        Returns:
            Workspace root path as string, or empty string on error.
        """
        try:
            from ament_index_python.packages import get_package_share_directory

            package_share_dir = get_package_share_directory("webvr_mocap")
            workspace_root = Path(package_share_dir).parent.parent.parent.parent
            return str(workspace_root)
        except (ImportError, KeyError, ValueError) as e:
            self.get_logger().error(
                f"Unable to get workspace root directory from package share directory: {e}"
            )
            return ""

    def start_vr_monitor(self) -> None:
        """Start the VR monitor in a separate thread."""
        workspace_root = self.get_workspace_root()
        if not workspace_root:
            self.get_logger().error("Cannot start VR monitor: workspace root not found")
            return

        webvr_mocap_src_path = os.path.join(
            workspace_root,
            "src",
            "driver",
            "webvr_mocap",
            "webvr_mocap",
            "vr",
        )
        self.get_logger().info(f"VR source path: {webvr_mocap_src_path}")

        try:
            self.vr_monitor = VRMonitor(webvr_mocap_src_path)
            self.get_logger().info("🚀 Starting VR monitoring...")
            vr_thread = Thread(
                target=lambda: asyncio.run(self.vr_monitor.start_monitoring()),
                daemon=True,
            )
            vr_thread.start()
            self.get_logger().info("VR system ready")
        except (OSError, RuntimeError, ValueError) as e:
            self.get_logger().error(f"Failed to start VR monitor: {e}")

    def receive_data(self) -> None:
        """Receive and process VR data, then publish transforms."""
        if self.vr_monitor is None:
            return

        goal = self.vr_monitor.get_latest_goal_nowait()
        if not self._validate_goal(goal):
            return

        # Publish joy data
        if "Joy" in goal:
            try:
                self.publish_joy(goal["Joy"])
            except (KeyError, TypeError, ValueError, IndexError) as e:
                self.get_logger().error(
                    f"Error publishing joy data: {type(e).__name__}: {e}. "
                    f"Goal data: {goal.get('Joy', 'N/A')}"
                )
            except Exception as e:
                self.get_logger().error(
                    f"Unexpected error publishing joy data: {type(e).__name__}: {e}. "
                    f"Goal data: {goal.get('Joy', 'N/A')}"
                )

        # Handle calibration
        self._handle_calibration(goal)

        # Process and publish transforms
        self._process_and_publish_transforms(goal)

    def _validate_goal(self, goal: Optional[Dict]) -> bool:
        """Validate that goal contains all required data.

        Args:
            goal: The goal dictionary to validate.

        Returns:
            True if goal is valid, False otherwise.
        """
        if goal is None:
            return False

        required_keys = ["Joy", "RightHand", "LeftHand", "Head"]
        return all(key in goal and goal[key] is not None for key in required_keys)

    def _handle_calibration(self, goal: Dict) -> None:
        """Handle calibration button presses.

        Args:
            goal: The goal dictionary containing controller data.
        """
        joy_data = goal.get("Joy", {})
        buttons = joy_data.get("buttons", [])

        if count_elements(buttons) != len(JoyBtnIndices):
            return

        # Calibrate on R_A button press
        if buttons[JoyBtnIndices.R_A][0] == 1 and not self.calibrated:
            self.calibrate(goal)

        # Reset calibration on R_B button press
        if buttons[JoyBtnIndices.R_B][0] == 1 and self.calibrated:
            self.robot_scale = DEFAULT_ROBOT_SCALE
            self.transform_offsets = {
                bone: TransformOffset() for bone in self.goal_bones
            }
            self.calibrated = False
            self.get_logger().info("Reset calibration to default")

    def _process_and_publish_transforms(self, goal: Dict) -> None:
        """Process VR data and publish transforms.

        Args:
            goal: The goal dictionary containing controller data.
        """
        current_time = self.get_clock().now().to_msg()

        for data, bone in zip(self._data, self.goal_bones):
            bone_data = goal[bone]

            # Extract position and quaternion
            pos = bone_data["position"]
            quat = bone_data["quaternion"]

            x, y, z = float(pos["x"]), float(pos["y"]), float(pos["z"])
            qx, qy, qz, qw = (
                float(quat["x"]),
                float(quat["y"]),
                float(quat["z"]),
                float(quat["w"]),
            )

            # Coordinate transformation from VR to robot frame
            data.transform.translation.x = -z
            data.transform.translation.y = -x
            data.transform.translation.z = y
            data.transform.rotation.x = -qz
            data.transform.rotation.y = -qx
            data.transform.rotation.z = qy
            data.transform.rotation.w = qw
            data.header.stamp = current_time

            # Apply offsets
            offset = self.transform_offsets[bone].position
            data.transform.translation.x -= offset.x
            data.transform.translation.y -= offset.y
            data.transform.translation.z += offset.z

            # Apply scale
            data.transform.translation.x *= self.robot_scale
            data.transform.translation.y *= self.robot_scale
            data.transform.translation.z *= self.robot_scale

            # Set frame ID based on calibration status
            data.child_frame_id = (
                f"{bone}_uncalibrated" if not self.calibrated else bone
            )

        self._pub.sendTransform(self._data)

    def calibrate(self, goal: Dict) -> None:
        """Calibrate the VR system based on current controller positions.

        Args:
            goal: The goal dictionary containing controller data.
        """
        right_arm_length = abs(float(goal["RightHand"]["position"]["z"]))
        left_arm_length = abs(float(goal["LeftHand"]["position"]["z"]))
        self.robot_scale = (
            0.5 * right_arm_length + 0.5 * left_arm_length
        ) / self.robot_arm_length

        for bone in self.transform_offsets.keys():
            vr_pos = goal[bone]["position"]
            offset = self.transform_offsets[bone].position
            offset.x = 0.0
            offset.y = 0.0
            offset.z = self.t_pos_dist[bone] - float(vr_pos["y"])

        self.get_logger().info(
            f"Calibrated: robot_scale={self.robot_scale:.3f}, "
            f"offsets={[(k, v.position.z) for k, v in self.transform_offsets.items()]}"
        )
        self.calibrated = True

    def publish_joy(self, joy_data: Dict) -> None:
        """Publish joy message from VR controller data.

        Args:
            joy_data: Dictionary containing axes and buttons data.
        """
        axes = joy_data.get("axes", [])
        buttons = joy_data.get("buttons", [])

        if not axes or not buttons:
            return

        axes_count = count_elements(axes)
        buttons_count = count_elements(buttons)

        if axes_count != len(JoyAxesIndices) or buttons_count != len(JoyBtnIndices):
            self.get_logger().error(
                f"Joy data length mismatch: axes={axes_count} (expected {len(JoyAxesIndices)}), "
                f"buttons={buttons_count} (expected {len(JoyBtnIndices)})"
            )
            return

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [float(axis) for axis in axes]
        # Flatten button array: [press, release] -> [press, release, ...]
        joy_msg.buttons = [int(btn[0]) for btn in buttons] + [
            int(btn[1]) for btn in buttons
        ]
        self.joy_pub.publish(joy_msg)


def main(args=None) -> None:
    """Main entry point for the webvr_mocap node."""
    rclpy.init(args=args)
    vr_mocap = WebVRMocap()
    try:
        rclpy.spin(vr_mocap)
    except KeyboardInterrupt:
        vr_mocap.get_logger().info("Shutting down webvr_mocap node...")
    finally:
        vr_mocap.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
