import rclpy
from rclpy.node import Node
import subprocess
import threading
from tf2_msgs.msg import TFMessage


class BagPlayerForwarder(Node):
    def __init__(self):
        super().__init__("bag_player_forwarder")
        self.declare_parameter("bag_path", "/rosbag2_2025_03_28-09_04_32")

        self.topic_map = {
            "/rosbag/temp/mocap/tf": "/mocap/tf",
        }

        self.bag_publishers = {}

        # 启动 ros2 bag play 子进程
        self.bag_process = self.start_bag_play(
            self.get_parameter("bag_path").value
        )  # 替换为你的 bag 路径

        # 为每个原始 topic 创建订阅者
        for src_topic in self.topic_map.keys():
            self.create_subscription(
                TFMessage,
                src_topic,
                lambda msg, src=src_topic: self.forward_callback(msg, src),
                10,
            )
            self.get_logger().info(f"Subscribed to {src_topic}")

    def start_bag_play(self, bag_path):
        cmd = [
            "ros2",
            "bag",
            "play",
            bag_path,
            "-l",
            "--remap",
            "/mocap/tf:=/rosbag/temp/mocap/tf",
        ]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def forward_callback(self, msg, src_topic):
        dest_topic = self.topic_map[src_topic]

        if dest_topic not in self.bag_publishers:
            self.bag_publishers[dest_topic] = self.create_publisher(
                type(msg), dest_topic, 10
            )
            self.get_logger().info(f"publisher {src_topic} -> {dest_topic}")

        for transform in msg.transforms:
            transform.header.stamp = self.get_clock().now().to_msg()
        self.bag_publishers[dest_topic].publish(msg)
        self.get_logger().debug(f"Forwarded: {src_topic} -> {dest_topic}")


def main(args=None):
    rclpy.init(args=args)
    node = BagPlayerForwarder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "bag_process"):
            node.bag_process.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
