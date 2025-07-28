import launch
from launch_ros.actions import Node


def generate_launch_description():
    ld = launch.LaunchDescription()

    # run adam_retarget node
    vr_tf_postprocess_node = Node(
        package="adam_retarget",
        executable="remove_timestamp.py",
        name="vr_tf_postprocess_node",
        output="screen",
        parameters=[{"topicfrom": "/tf", "topicto": "/vr/tf"}],
    )
    ld.add_action(vr_tf_postprocess_node)

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
        remappings=[("/tf", "/vr/tf"), ("/tf_static", "/vr/tf_static")],
    )
    ld.add_action(static_transform_publisher_node)

    return ld
