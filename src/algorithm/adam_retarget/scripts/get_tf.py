#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
import yaml
import os

dump_index = 0


def main():
    rclpy.init()
    node = Node("tf_dumper")

    dump_path = (
        node.declare_parameter(
            "dump_path",
            value="tf_dump",
        )
        .get_parameter_value()
        .string_value
    )

    world_frame = (
        node.declare_parameter(
            "world_frame",
            value="world",
        )
        .get_parameter_value()
        .string_value
    )

    if not os.path.exists(dump_path):
        os.makedirs(dump_path)
    print(f"Dumping frames to {dump_path}")

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
        td_file_name = f"{dump_index}.txt"
        with open(os.path.join(dump_path, td_file_name), "w") as f:
            for frame_id in frames_dict:
                if frame_id == world_frame:
                    continue
                try:
                    trans = tf_buffer.lookup_transform(
                        world_frame, frame_id, rclpy.time.Time(), Duration(seconds=1.0)
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
