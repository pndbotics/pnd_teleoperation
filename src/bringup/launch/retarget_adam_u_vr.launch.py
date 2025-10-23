import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # urdf
    package_name = "adam_description"
    urdf_name = "urdf/adam_u/urdf/adam_u.urdf"
    urdf_pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(urdf_pkg_share, urdf_name)

    with open(urdf_model_path, "r") as infp:
        robot_desc = infp.read()
    robot_state_publisher_node = Node(
        package="adam_state_publisher",
        executable="adam_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    # rviz
    bringup_pkg_share = FindPackageShare(package="bringup").find("bringup")
    rviz_config_file = "rviz/robot.rviz"
    rviz_config_file_path = os.path.join(bringup_pkg_share, rviz_config_file)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file_path],
    )

    adam_reterget_pkg_share = FindPackageShare(package="adam_retarget").find(
        "adam_retarget"
    )
    noitom_sp_json_path = os.path.join(
        adam_reterget_pkg_share,
        "opti_config/meta_vr/adam_u/Adam_U_Meta_VR_Deploy_opti.json",
    )
    adam_retarget_node = Node(
        package="adam_retarget",
        executable="adam_retarget",
        name="adam_retarget",
        output="screen",
        parameters=[
            {"base_frame": "world"},
            {"control_loop_rate": 100.0},
            {"config_json_path": noitom_sp_json_path},
        ],
        remappings=[("/tf", "/mocap/tf"), ("/tf_static", "/mocap/tf_static")],
    )

    vr_mocap = Node(
        package="vr_mocap",
        executable="vr_mocap",
        name="vr_robot_tf_broadcaster",
        remappings=[("/tf", "/mocap/tf")],
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(adam_retarget_node)
    ld.add_action(rviz_node)
    ld.add_action(vr_mocap)

    return ld
