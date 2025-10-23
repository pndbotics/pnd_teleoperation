import rclpy
from rclpy.node import Node
import socket
import json
from geometry_msgs.msg import TransformStamped, Transform

from tf2_ros import TransformBroadcaster
import enum
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import Trigger
from dataclasses import dataclass


class EOculusXRBoneId(enum.IntEnum):
    BodyRoot = (0,)
    BodyHips = (1,)
    BodySpineLower = (2,)
    BodySpineMiddle = (3,)
    BodySpineUpper = (4,)
    BodyChest = (5,)
    BodyNeck = (6,)
    BodyHead = (7,)
    BodyLeftShoulder = (8,)
    BodyLeftScapula = (9,)
    BodyLeftArmUpper = (10,)
    BodyLeftArmLower = (11,)
    BodyLeftHandWristTwist = (12,)
    BodyRightShoulder = (13,)
    BodyRightScapula = (14,)
    BodyRightArmUpper = (15,)
    BodyRightArmLower = (16,)
    BodyRightHandWristTwist = (17,)
    BodyLeftHandPalm = (18,)
    BodyLeftHandWrist = (19,)
    BodyLeftHandThumbMetacarpal = (20,)
    BodyLeftHandThumbProximal = (21,)
    BodyLeftHandThumbDistal = (22,)
    BodyLeftHandThumbTip = (23,)
    BodyLeftHandIndexMetacarpal = (24,)
    BodyLeftHandIndexProximal = (25,)
    BodyLeftHandIndexIntermediate = (26,)
    BodyLeftHandIndexDistal = (27,)
    BodyLeftHandIndexTip = (28,)
    BodyLeftHandMiddleMetacarpal = (29,)
    BodyLeftHandMiddleProximal = (30,)
    BodyLeftHandMiddleIntermediate = (31,)
    BodyLeftHandMiddleDistal = (32,)
    BodyLeftHandMiddleTip = (33,)
    BodyLeftHandRingMetacarpal = (34,)
    BodyLeftHandRingProximal = (35,)
    BodyLeftHandRingIntermediate = (36,)
    BodyLeftHandRingDistal = (37,)
    BodyLeftHandRingTip = (38,)
    BodyLeftHandLittleMetacarpal = (39,)
    BodyLeftHandLittleProximal = (40,)
    BodyLeftHandLittleIntermediate = (41,)
    BodyLeftHandLittleDistal = (42,)
    BodyLeftHandLittleTip = (43,)
    BodyRightHandPalm = (44,)
    BodyRightHandWrist = (45,)
    BodyRightHandThumbMetacarpal = (46,)
    BodyRightHandThumbProximal = (47,)
    BodyRightHandThumbDistal = (48,)
    BodyRightHandThumbTip = (49,)
    BodyRightHandIndexMetacarpal = (50,)
    BodyRightHandIndexProximal = (51,)
    BodyRightHandIndexIntermediate = (52,)
    BodyRightHandIndexDistal = (53,)
    BodyRightHandIndexTip = (54,)
    BodyRightHandMiddleMetacarpal = (55,)
    BodyRightHandMiddleProximal = (56,)
    BodyRightHandMiddleIntermediate = (57,)
    BodyRightHandMiddleDistal = (58,)
    BodyRightHandMiddleTip = (59,)
    BodyRightHandRingMetacarpal = (60,)
    BodyRightHandRingProximal = (61,)
    BodyRightHandRingIntermediate = (62,)
    BodyRightHandRingDistal = (63,)
    BodyRightHandRingTip = (64,)
    BodyRightHandLittleMetacarpal = (65,)
    BodyRightHandLittleProximal = (66,)
    BodyRightHandLittleIntermediate = (67,)
    BodyRightHandLittleDistal = (68,)
    BodyRightHandLittleTip = (69,)
    BodyLeftUpperLeg = (70,)
    BodyLeftLowerLeg = (71,)
    BodyLeftFootAnkleTwist = (72,)
    BodyLeftFootAnkle = (73,)
    BodyLeftFootSubtalar = (74,)
    BodyLeftFootTransverse = (75,)
    BodyLeftFootBall = (76,)
    BodyRightUpperLeg = (77,)
    BodyRightLowerLeg = (78,)
    BodyRightFootAnkleTwist = (79,)
    BodyRightFootAnkle = (80,)
    BodyRightFootSubtalar = (81,)
    BodyRightFootTransverse = (82,)
    BodyRightFootBall = 83


@dataclass
class TFLimit:
    body_id: list[EOculusXRBoneId]
    velocity_limit: float
    angular_velocity_limit: float


limit_config = {
    "body": TFLimit(
        body_id=[
            EOculusXRBoneId.BodyRoot,
            EOculusXRBoneId.BodyHips,
            EOculusXRBoneId.BodySpineLower,
            EOculusXRBoneId.BodySpineMiddle,
            EOculusXRBoneId.BodySpineUpper,
            EOculusXRBoneId.BodyChest,
        ],
        velocity_limit=1,  # m/s
        angular_velocity_limit=2.0,  # rad/s
    ),
    "arm": TFLimit(
        body_id=[
            EOculusXRBoneId.BodyLeftShoulder,
            EOculusXRBoneId.BodyLeftScapula,
            EOculusXRBoneId.BodyLeftArmUpper,
            EOculusXRBoneId.BodyLeftArmLower,
            EOculusXRBoneId.BodyRightShoulder,
            EOculusXRBoneId.BodyRightScapula,
            EOculusXRBoneId.BodyRightArmUpper,
            EOculusXRBoneId.BodyRightArmLower,
        ],
        velocity_limit=1.5,  # m/s
        angular_velocity_limit=3.0,  # rad/s
    ),
}

ip_specific_limit: dict[int, TFLimit] = {}
for group, limit in limit_config.items():
    for bone_id in limit.body_id:
        ip_specific_limit[bone_id] = limit


def calculate_velocity(
    previous_transform: Transform, current_transform: Transform, dt: float
):
    distance = (
        (current_transform.translation.x - previous_transform.translation.x) ** 2
        + (current_transform.translation.y - previous_transform.translation.y) ** 2
        + (current_transform.translation.z - previous_transform.translation.z) ** 2
    ) ** 0.5
    rotation_previous = R.from_quat(
        [
            previous_transform.rotation.x,
            previous_transform.rotation.y,
            previous_transform.rotation.z,
            previous_transform.rotation.w,
        ]
    )
    rotation_current = R.from_quat(
        [
            current_transform.rotation.x,
            current_transform.rotation.y,
            current_transform.rotation.z,
            current_transform.rotation.w,
        ]
    )
    angular_distance = rotation_previous.inv() * rotation_current
    angular_distance = angular_distance.magnitude()
    velocity = distance / dt
    angular_velocity = angular_distance / dt
    return velocity, angular_velocity


class VRMocap(Node):
    def __init__(self):
        super().__init__("vr_mocap")
        self._data: list[TransformStamped] = []
        for bone in EOculusXRBoneId:
            stamped = TransformStamped()
            stamped.header.frame_id = "world"
            stamped.child_frame_id = str(bone.name)
            stamped.transform.translation.x = 0.0
            stamped.transform.translation.y = 0.0
            stamped.transform.translation.z = 0.0
            stamped.transform.rotation.x = 0.0
            stamped.transform.rotation.y = 0.0
            stamped.transform.rotation.z = 0.0
            stamped.transform.rotation.w = 1.0
            self._data.append(stamped)

        self._pub = TransformBroadcaster(self)
        self._robot_protect_client = self.create_client(
            Trigger, "/adam_retarget/warm_start"
        )
        while not self._robot_protect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for /adam_retarget/warm_start service...",
                throttle_duration_sec=1.0,
            )
        self.get_logger().info(
            "Connected to /adam_retarget/warm_start service, calling warm start..."
        )
        self.udp_ip = "0.0.0.0"
        self.udp_port = 12069
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(
            f"UDP receiver listening on {self.udp_ip}:{self.udp_port}"
        )
        self.receive_data()

    def receive_data(self):
        while rclpy.ok():
            data, addr = self.sock.recvfrom(1024 * 20)
            time_stamp = self.get_clock().now()
            # self.get_logger().info(f"Received message: {data.decode()} from {addr}")
            recv_j = json.loads(data.decode())
            if len(recv_j.keys()) != 84:
                self.get_logger().warn("Received message is not complete!")
                break
            protect_flag = False
            for idx_s, pose in recv_j.items():
                idx = int(idx_s)
                x, y, z, qx, qy, qz, qw = map(float, pose)
                # Unreal Engine uses left-handed coordinate system
                # x : forward, y : right, z : up
                # ROS uses right-handed coordinate system
                # x : forward, y : left, z : up
                # Convert Unreal Engine quaternion to ROS quaternion
                new_frame = Transform()
                new_frame.translation.x = x * 0.01
                new_frame.translation.y = -y * 0.01
                new_frame.translation.z = z * 0.01

                new_frame.rotation.x = -qx
                new_frame.rotation.y = qy
                new_frame.rotation.z = -qz
                new_frame.rotation.w = qw

                # Calculate velocity and angular velocity
                if not protect_flag and idx in ip_specific_limit:
                    ros_now = time_stamp.to_msg()
                    ros_prev = self._data[idx].header.stamp
                    dt = (ros_now.sec - ros_prev.sec) + (
                        ros_now.nanosec - ros_prev.nanosec
                    ) * 1e-9
                    lin_vel, ang_vel = calculate_velocity(
                        self._data[idx].transform, new_frame, dt
                    )
                    if (
                        lin_vel > ip_specific_limit[idx].velocity_limit
                        or ang_vel > ip_specific_limit[idx].angular_velocity_limit
                    ):
                        self.get_logger().warn(
                            f"Velocity limit exceeded for {EOculusXRBoneId(idx).name}: "
                            f"linear velocity {lin_vel:.2f} m/s, "
                            f"angular velocity {ang_vel:.2f} rad/s"
                        )
                        protect_flag = True

                self._data[idx].transform = new_frame
                self._data[idx].header.stamp = time_stamp.to_msg()

            if protect_flag:
                self.get_logger().warn(
                    "Robot protection triggered due to velocity limit exceeded.",
                    throttle_duration_sec=1.5,
                )
                self.call_warm_start()
            else:
                self._pub.sendTransform(self._data)

    def call_warm_start(self):
        req = Trigger.Request()

        future = self._robot_protect_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(
                f"Warm start response: {future.result().success}, {future.result().message}"
            )
        else:
            self.get_logger().error("Service call failed")


def main(args=None):
    rclpy.init(args=args)
    vr_mocap = VRMocap()
    rclpy.spin(vr_mocap)
    vr_mocap.sock.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
