import rclpy
from rclpy.node import Node
import socket
import json
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import enum
from scipy.spatial.transform import Rotation as R


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


class VRMocap(Node):
    def __init__(self):
        super().__init__("vr_mocap")
        self._data = []
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
            # self.get_logger().info(f"Received message: {data.decode()} from {addr}")
            recv_j = json.loads(data.decode())
            if len(recv_j.keys()) != 84:
                self.get_logger().warn("Received message is not complete!")
                break
            for idx_s, val in recv_j.items():
                idx = int(idx_s)
                self._data[idx].transform.translation.x = float(val[0]) / 100.0
                self._data[idx].transform.translation.y = -float(val[1]) / 100.0
                self._data[idx].transform.translation.z = float(val[2]) / 100.0

                r = R.from_euler(
                    "zyx", [float(val[5]), -float(val[4]), float(val[3])], degrees=True
                )
                quaternion = r.as_quat()
                self._data[idx].transform.rotation.x = quaternion[0]
                self._data[idx].transform.rotation.y = quaternion[1]
                self._data[idx].transform.rotation.z = quaternion[2]
                self._data[idx].transform.rotation.w = quaternion[3]

            for _ in self._data:
                _.header.stamp = self.get_clock().now().to_msg()
            self._pub.sendTransform(self._data)


def main(args=None):
    rclpy.init(args=args)
    vr_mocap = VRMocap()
    rclpy.spin(vr_mocap)
    vr_mocap.sock.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
