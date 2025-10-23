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
        default_value="foxglove",
        description="Choose visualization tool: rviz2 or foxglove",
        choices=["rviz2", "foxglove"],
    )
    ld.add_entity(visual_declare)

    # urdf
    package_name = "adam_description"
    urdf_name = "urdf/adam_sp/urdf_plus/standard_plus53.urdf"
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

    # transform noitom y up to z up
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
        remappings=[("/tf", "/rosbag/mocap/tf"), ("/tf_static", "/mocap/tf_static")],
    )

    adam_reterget_pkg_share = FindPackageShare(package="adam_retarget").find(
        "adam_retarget"
    )
    noitom_sp_json_path = os.path.join(
        adam_reterget_pkg_share,
        "opti_config/adam_sp/Adam_SP_Single_Noitom_Hand_By_Pos_Debug_opti.json",
    )
    adam_retarget_node = Node(
        package="adam_retarget",
        executable="adam_retarget",
        name="adam_retarget",
        output="screen",
        parameters=[
            {
                "base_frame": "world_zup",
                "control_loop_rate": 100.0,
                "config_json_path": noitom_sp_json_path,
                "custom_weight": {
                    # "head_dir_weight": 5.0,
                    # "root_pos_weight": 100.0,
                    # "root_dir_weight": 100.0,
                    # "torso_dir_weight": 50.0,
                    # "shoulder_dir_weight": 30.0,
                    # "upper_arm_weight": 20.0,
                    # "L_upper_arm_weight": 20.0,
                    # "R_upper_arm_weight": 20.0,
                    # "lower_arm_weight": 10.0,
                    # "L_lower_arm_weight": 10.0,
                    # "R_lower_arm_weight": 10.0,
                    # "hand_dir_front_weight": 5.0,
                    # "hand_dir_outside_weight": 3.0,
                    # "finger_weight": 10.0,
                    # "upper_leg_weight": 50.0,
                    # "lower_leg_weight": 30.0,
                    # "foot_pos_weight": 10.0,
                    # "hand_pos_weight": 10.0,
                    # "L_hand_pos_weight": 10.0,
                    # "R_hand_pos_weight": 10.0,
                    # "foot_dir_x_weight": 20.0,
                    # "foot_dir_y_weight": 0.0,
                    # "joint_vel_cost_weight": 0.0010000000474974513,
                    # "default_config_cost_weight": 0.0010000000474974513,
                    # "finger_turbo_factor": 1.0,
                    # "thumb_pos_weight": 10.0,
                },
            },
        ],
        remappings=[("/tf", "/rosbag/mocap/tf"), ("/tf_static", "/mocap/tf_static")],
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(adam_retarget_node)
    ld.add_action(static_transform_publisher_node)

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
