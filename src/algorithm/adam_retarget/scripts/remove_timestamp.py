#!/usr/bin/env python3

# ROS2 sub to tf topic1 and publish tf topic2, flush timestamp to now

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def quat_mult(q1_x, q1_y, q1_z, q1_w, q2_x, q2_y, q2_z, q2_w):
    x = q1_w * q2_x + q1_x * q2_w + q1_y * q2_z - q1_z * q2_y
    y = q1_w * q2_y - q1_x * q2_z + q1_y * q2_w + q1_z * q2_x
    z = q1_w * q2_z + q1_x * q2_y - q1_y * q2_x + q1_z * q2_w
    w = q1_w * q2_w - q1_x * q2_x - q1_y * q2_y - q1_z * q2_z
    return x, y, z, w


class TfRepublisher(Node):
    def __init__(self):
        super().__init__("tf_republisher")

        # get parameters
        self.declare_parameter("topicfrom", "/tf")
        self.declare_parameter("topicto", "/notimestamp/tf")
        topicfrom = self.get_parameter("topicfrom").value
        topicto = self.get_parameter("topicto").value

        self.tf_sub = self.create_subscription(
            TFMessage, topicfrom, self.tf_callback, 10
        )

        self.tf_pub = self.create_publisher(TFMessage, topicto, 10)

    def handle_tf(self, msg: TFMessage):
        for transform in msg.transforms:
            transform.header.stamp = self.get_clock().now().to_msg()
            # rotated_quat = quat_mult(
            #     transform.transform.rotation.x,
            #     transform.transform.rotation.y,
            #     transform.transform.rotation.z,
            #     transform.transform.rotation.w,
            #     0.7071067811865476,
            #     0,
            #     0,
            #     0.7071067811865476
            # )
            # transform.transform.rotation.x = rotated_quat[0]
            # transform.transform.rotation.y = rotated_quat[1]
            # transform.transform.rotation.z = rotated_quat[2]
            # transform.transform.rotation.w = rotated_quat[3]

        self.tf_pub.publish(msg)

    def tf_callback(self, msg):
        self.handle_tf(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
