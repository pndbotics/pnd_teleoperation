import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "noitom_mocap"
    ld = LaunchDescription()

    noitom_mocap = Node(
        package="noitom_mocap",
        executable="noitom_mocap",
        name="noitom_robot_tf_broadcaster",
        remappings=[("/tf", "/noitom/tf")],
    )
    ld.add_action(noitom_mocap)
    return ld
