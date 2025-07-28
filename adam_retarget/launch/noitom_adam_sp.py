import launch
from launch_ros.actions import Node


def generate_launch_description():
    # run adam_retarget node
    noitom_sp_json_path = "/pnd_workspace/adam_retarget/opti_config/adam_sp/Adam_SP_Single_Noitom_Debug_opti.json"
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
        remappings=[("/tf", "/mocap/tf")],
    )

    return launch.LaunchDescription([adam_retarget_node])
