from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
import os


def generate_launch_description():

    bag_path_arg = DeclareLaunchArgument(
        name="bag",
        description="Full path of ros bag.",
        default_value="",
    )

    dump_folder_arg = DeclareLaunchArgument(
        name="dump_folder",
        description="Output dir.",
        default_value="tf_dump",
    )

    bag_play_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag"),
            "--remap",
            "/mocap/tf:=/tf",
        ],
        output="screen",
    )

    dump_tf = Node(
        package="adam_retarget",
        executable="get_tf.py",
        name="tf_dumper",
        output="screen",
        parameters=[{"dump_path": LaunchConfiguration("dump_folder")}],
    )
    return LaunchDescription(
        [
            bag_path_arg,
            dump_folder_arg,
            bag_play_process,
            dump_tf,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=bag_play_process,
                    on_exit=[Shutdown()],
                )
            ),
        ]
    )
