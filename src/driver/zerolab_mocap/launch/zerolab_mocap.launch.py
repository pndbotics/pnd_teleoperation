import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    visual_declare = DeclareLaunchArgument(
        "visual",
        default_value="rviz2",
        description="Choose visualization tool: rviz2 or foxglove",
        choices=["rviz2", "foxglove"],
    )
    ld.add_entity(visual_declare)

    # transform zerolab y up to z up
    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0.707106781",
            "0",
            "0",
            "0.707106781",
            "world_zup",
            "world",
        ],
        # remappings=[("/tf", "/mocap/tf"), ("/tf_static", "/mocap/tf_static")],
    )
    zerolab_mocap = Node(
        package="zerolab_mocap",
        executable="zerolab_mocap",
        name="zerolab_tf_broadcaster",
        # remappings=[
        #     ("/tf", "/mocap/tf"),
        #     ("/joint_states_passthrough", "/zerolab_finger_states"),
        # ],
    )

    ld.add_action(zerolab_mocap)
    # ld.add_action(static_transform_publisher_node)

    bringup_pkg_share = FindPackageShare(package="bringup").find("bringup")
    rviz_config_file = "rviz/robot.rviz"
    rviz_config_file_path = os.path.join(bringup_pkg_share, rviz_config_file)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file_path],
        condition=LaunchConfigurationEquals("visual", "rviz2"),
    )
    ld.add_action(rviz_node)

    return ld
