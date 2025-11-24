import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    visual_declare = DeclareLaunchArgument(
        "visual",
        default_value="foxglove",
        description="Choose visualization tool: rviz2 or foxglove",
        choices=["rviz2", "foxglove"],
    )
    ld.add_entity(visual_declare)

    # urdf
    package_name = "adam_description"
    urdf_name = "urdf/adam_u/adam_u.urdf"
    urdf_pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(urdf_pkg_share, urdf_name)
    mjcf_model_path = os.path.join(urdf_pkg_share, "urdf/adam_u/adam_u.xml")

    with open(urdf_model_path, "r") as infp:
        robot_desc = infp.read()
    robot_state_publisher_node = Node(
        package="adam_state_publisher",
        executable="adam_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "root_link": "lifting_Columns"}],
    )

    adam_mink_path = FindPackageShare(package="adam_mink").find("adam_mink")
    ik_cfg_path = os.path.join(
        adam_mink_path,
        "config/adam_u_vr_ik_config.yaml",
    )
    adam_mink_node = Node(
        package="adam_mink",
        executable="adam_mink",
        name="adam_mink",
        parameters=[
            {"adam_mink_cfg": ik_cfg_path},
            {"adam_model_path": mjcf_model_path},
            {"mujoco_sim": False},
        ],
        remappings=[
            ("/tf", "/mocap/tf"),
            ("/tf_static", "/mocap/tf_static"),
        ],
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(adam_mink_node)

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

    webvr_mocap_node = Node(
        package="webvr_mocap",
        executable="webvr_mocap",
        name="webvr_mocap",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("/tf", "/mocap/tf"),
        ],
    )
    ld.add_action(webvr_mocap_node)

    return ld
