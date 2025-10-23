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
    urdf_name = "urdf/adam_u/adam_u.urdf"
    urdf_pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(urdf_pkg_share, urdf_name)

    with open(urdf_model_path, "r") as infp:
        robot_desc = infp.read()
    robot_state_publisher_node = Node(
        package="adam_state_publisher",
        executable="adam_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "root_link": "lifting_Columns"}],
    )
    
    adam_reterget_pkg_share = FindPackageShare(package="adam_retarget").find(
        "adam_retarget"
    )
    noitom_sp_json_path = os.path.join(
        adam_reterget_pkg_share,
        "opti_config/pteleop/threepoint/adam_u/Adam_U_ThreePoint_Deploy_opti.json",
    )
    # arm_joint_vel_cost_weight: 0.001000
    # default_config_cost_weight: 0.100000
    # hand_dir_front_weight: 5.000000
    # hand_dir_outside_weight: 3.000000
    # hand_joint_vel_cost_weight: 0.000000
    # hand_pos_weight: 10.000000
    # head_dir_weight: 5.000000
    # root_pos_penalty_weight: 50.000000
    # root_pos_weight: 100.000000
    # shoulder_roll_default_cost_weight: 0.001000
    # waist_joint_vel_cost_weight: 0.000000

    adam_retarget_node = Node(
        package="adam_retarget",
        executable="adam_retarget",
        name="adam_retarget",
        output="screen",
        parameters=[
            {"base_frame": "world"},
            {"control_loop_rate": 100.0},
            {"config_json_path": noitom_sp_json_path},
            {
                "warm_start_trig_timeout": 0.1,  # if mocap data is not received in this time, retarget will trigger warm start
                "warm_start_duration": 3.0,  # in next 5 seconds after receiving new mocap data, retarget will use warm start velocity
                "warm_start_slowdown_ratio": 0.1,  # when retarget is using warm start velocity, the velocity will be multiplied by this ratio, usually much smaller than 1.0}
            },
            {
                "custom_weight": {
                    "hand_pos_weight": 100.0,
                    "arm_joint_vel_cost_weight": 0.00001000,
                    "root_pos_penalty_weight": 100.0,
                    "elbow_steady_cost_weight": 50.0,
                    "hand_dir_front_weight": 0.5,
                    "hand_dir_outside_weight": 0.5,
                }
            },
        ],
        remappings=[
            ("/tf", "/mocap/tf"),
            ("/tf_static", "/mocap/tf_static"),
        ],
    )

    # pteleop_bridge node
    pteleop_bridge_node = Node(
        package="pteleop_bridge",
        executable="pteleop_bridge_node",
        name="pteleop_bridge",
        parameters=[{"server_port": 12069}],
        remappings=[
            ("/tf", "/mocap/tf"),
        ],
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(adam_retarget_node)

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
    ld.add_action(pteleop_bridge_node)

    return ld
