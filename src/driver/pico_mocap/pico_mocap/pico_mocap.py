import json
import socket
import threading
from dataclasses import dataclass, field
from typing import ClassVar

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from shared_utils.shared_utils import JoyAxesIndices, PicoJoyBtnIndices
from tf2_ros import TransformBroadcaster

from pico_mocap.utils import (
    _mat_mul,
    _mat_vec_mul,
    _matrix_to_quat,
    _quat_to_matrix,
)


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


class PicoMocap(Node):
    BODY_NAMES: ClassVar[tuple[str, ...]] = (
        "pelvis",
        "l-hip",
        "r-hip",
        "spine-01",
        "l-knee",
        "r-knee",
        "spine-02",
        "l-ankle",
        "r-ankle",
        "spine-03",
        "l-foot",
        "r-foot",
        "neck",
        "l-collar",
        "r-collar",
        "head",
        "l-shoulder",
        "r-shoulder",
        "l-elbow",
        "r-elbow",
        "l-wrist",
        "r-wrist",
        "l-hand",
        "r-hand",
    )

    def __init__(self) -> None:
        super().__init__("pico_mocap")

        self._frames: dict[str, TransformStamped] = {}
        for name in self.BODY_NAMES:
            stamped = TransformStamped()
            stamped.header.frame_id = "world"
            stamped.child_frame_id = name
            stamped.transform.translation.x = 0.0
            stamped.transform.translation.y = 0.0
            stamped.transform.translation.z = 0.0
            stamped.transform.rotation.x = 0.0
            stamped.transform.rotation.y = 0.0
            stamped.transform.rotation.z = 0.0
            stamped.transform.rotation.w = 1.0
            self._frames[name] = stamped

        self._tf_broadcaster = TransformBroadcaster(self)
        self._joy_pub = self.create_publisher(Joy, "pico/joy", 10)

        # PICO uses Unity-style coordinates (same as Vive/SteamVR):
        # Unity: x right, y up, z forward
        # Robot: x forward, y left, z up
        self._unity_to_robot = [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]
        self._unity_to_robot_T = [
            [
                self._unity_to_robot[0][0],
                self._unity_to_robot[1][0],
                self._unity_to_robot[2][0],
            ],
            [
                self._unity_to_robot[0][1],
                self._unity_to_robot[1][1],
                self._unity_to_robot[2][1],
            ],
            [
                self._unity_to_robot[0][2],
                self._unity_to_robot[1][2],
                self._unity_to_robot[2][2],
            ],
        ]
        self.robot_scale = 1.0

        self.udp_ip = "0.0.0.0"
        self.udp_port = 12070
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(
            f"PICO mocap UDP receiver listening on {self.udp_ip}:{self.udp_port}"
        )

        self._recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self._recv_thread.start()

    def _get_frame_pose(self, name: str) -> tuple[list[float], list[float]]:
        frame = self._frames[name].transform
        t_in = [frame.translation.x, frame.translation.y, frame.translation.z]
        q_in = [frame.rotation.x, frame.rotation.y, frame.rotation.z, frame.rotation.w]
        return t_in, q_in

    def _set_frame_pose(self, name: str, t: list[float], q: list[float]) -> None:
        frame = self._frames[name].transform
        frame.translation.x = float(t[0])
        frame.translation.y = float(t[1])
        frame.translation.z = float(t[2])
        frame.rotation.x = float(q[0])
        frame.rotation.y = float(q[1])
        frame.rotation.z = float(q[2])
        frame.rotation.w = float(q[3])

    def _apply_root_relative_scaling(self, names: list[str], root_name: str = "pelvis") -> None:
        """Scale joint positions around a fixed root, preserving pose structure."""
        root_frame = self._frames.get(root_name)
        if root_frame is None:
            return

        root = np.array(
            [
                root_frame.transform.translation.x,
                root_frame.transform.translation.y,
                root_frame.transform.translation.z,
            ],
            dtype=float,
        )

        for name in names:
            frame = self._frames.get(name)
            if frame is None:
                continue

            p = np.array(
                [
                    frame.transform.translation.x,
                    frame.transform.translation.y,
                    frame.transform.translation.z,
                ],
                dtype=float,
            )
            p_scaled = root + self.robot_scale * (p - root)
            frame.transform.translation.x = float(p_scaled[0])
            frame.transform.translation.y = float(p_scaled[1])
            frame.transform.translation.z = float(p_scaled[2])

    def _update_frame(
        self,
        name: str,
        position: dict[str, float],
        orientation: dict[str, float],
    ) -> None:
        if name not in self._frames:
            stamped = TransformStamped()
            stamped.header.frame_id = "world"
            stamped.child_frame_id = name
            self._frames[name] = stamped

        frame = self._frames[name]

        x, y, z = [float(i) for i in position.split(",")]
        qx, qy, qz, qw = [float(i) for i in orientation.split(",")]

        # Position: map Unity VR -> robot frame using a fixed 3x3 transform.
        rx, ry, rz = _mat_vec_mul(self._unity_to_robot, [x, y, z])
        frame.transform.translation.x = rx
        frame.transform.translation.y = ry
        frame.transform.translation.z = rz

        # Orientation: convert quaternion -> matrix, change basis with the same
        # Unity -> robot transform, then convert back to quaternion.
        ru = _quat_to_matrix(qx, qy, qz, qw)

        # R_robot = P * R_unity * P^T
        p = self._unity_to_robot
        rr = _mat_mul(_mat_mul(p, ru), self._unity_to_robot_T)
        rqx, rqy, rqz, rqw = _matrix_to_quat(rr)

        frame.transform.rotation.x = rqx
        frame.transform.rotation.y = rqy
        frame.transform.rotation.z = rqz
        frame.transform.rotation.w = rqw

    def _publish_joy_from_controller(self, controller: dict, time_stamp) -> None:
        """Map PICO `Controller` dict to `sensor_msgs/Joy` (same layout as webvr_mocap)."""
        left = controller.get("left") or {}
        right = controller.get("right") or {}

        axes = [0.0] * len(JoyAxesIndices)
        axes[JoyAxesIndices.L_x] = float(left.get("axisX", 0.0))
        axes[JoyAxesIndices.L_y] = float(left.get("axisY", 0.0))
        axes[JoyAxesIndices.L_trigger] = float(left.get("trigger", 0.0))
        axes[JoyAxesIndices.L_grip] = float(left.get("grip", 0.0))
        axes[JoyAxesIndices.R_x] = float(right.get("axisX", 0.0))
        axes[JoyAxesIndices.R_y] = float(right.get("axisY", 0.0))
        axes[JoyAxesIndices.R_trigger] = float(right.get("trigger", 0.0))
        axes[JoyAxesIndices.R_grip] = float(right.get("grip", 0.0))

        def _digital_btn(raw: object) -> int:
            if isinstance(raw, bool):
                return 1 if raw else 0
            if isinstance(raw, (int, float)):
                return 1 if float(raw) > 0.5 else 0
            return 0

        press = [0] * len(PicoJoyBtnIndices)
        press[PicoJoyBtnIndices.L_AxisClick] = _digital_btn(left.get("axisClick", False))
        press[PicoJoyBtnIndices.L_X] = _digital_btn(left.get("primaryButton", False))
        press[PicoJoyBtnIndices.L_Y] = _digital_btn(left.get("secondaryButton", False))
        press[PicoJoyBtnIndices.R_AxisClick] = _digital_btn(right.get("axisClick", False))
        press[PicoJoyBtnIndices.R_A] = _digital_btn(right.get("primaryButton", False))
        press[PicoJoyBtnIndices.R_B] = _digital_btn(right.get("secondaryButton", False))

        joy_msg = Joy()
        joy_msg.header.stamp = time_stamp.to_msg()
        joy_msg.axes = axes
        joy_msg.buttons = [int(x) for x in press]
        self._joy_pub.publish(joy_msg)

    def receive_loop(self) -> None:
        while rclpy.ok():
            # UDP max payload is 65507 bytes; Pico whole-body JSON can exceed 4KB.
            data, addr = self.sock.recvfrom(1024 * 4)
            time_stamp = self.get_clock().now()
            try:
                payload = json.loads(data.decode())
                # self.get_logger().info(f"Received message: {payload}")
            except Exception as exc:
                self.get_logger().warn(f"Failed to decode JSON from {addr}: {exc}")
                self.get_logger().warn(f"Data: {data}")
                continue

            self._process_payload(payload, time_stamp)

    def _process_payload(self, payload: dict, time_stamp) -> None:
        """Common processing for a single PICO payload dict."""
        bones = payload.get("Body", {}).get("joints", [])
        updated_names: list[str] = []
        for bone in bones:
            self._update_frame(bone["bone"], bone["pos"], bone["rot"])
            updated_names.append(bone["bone"])
            self._frames[bone["bone"]].header.stamp = time_stamp.to_msg()

        self._apply_root_relative_scaling(updated_names)
        self._tf_broadcaster.sendTransform(list(self._frames.values()))

        controller = payload.get("Controller")
        if isinstance(controller, dict):
            self._publish_joy_from_controller(controller, time_stamp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PicoMocap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting pico_mocap down...")
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
