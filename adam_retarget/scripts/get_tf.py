#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
import yaml

dump_index = 0


def main():
    rclpy.init()
    node = Node("tf_dumper")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    def dump_frames():
        global dump_index
        frames_str = tf_buffer.all_frames_as_yaml()
        frames_dict = yaml.safe_load(frames_str) or {}

        if len(frames_dict) == 0:
            node.get_logger().info("No frames found.")
            return

        dump_index += 1
        td_file_name = f"tf_dump_{dump_index}.txt"
        with open(td_file_name, "w") as f:
            for frame_id in frames_dict:
                if frame_id == "world":
                    continue
                try:
                    trans = tf_buffer.lookup_transform(
                        "world", frame_id, rclpy.time.Time(), Duration(seconds=1.0)
                    )
                    pos = trans.transform.translation
                    rot = trans.transform.rotation
                    f.write(
                        f"{frame_id}:({rot.x},{rot.y},{rot.z},{rot.w}) ({pos.x},{pos.y},{pos.z})\n"
                    )
                except (LookupException, ConnectivityException, ExtrapolationException):
                    pass
        node.get_logger().info("Frames dumped.")

    node.create_timer(0.02, dump_frames)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
