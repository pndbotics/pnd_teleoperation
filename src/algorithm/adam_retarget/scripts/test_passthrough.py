import rclpy
from sensor_msgs.msg import JointState

# create a ROS2 node
rclpy.init()
node = rclpy.create_node("test_passthrough")

# create a publisher to publish to /joint_states_passthrough
publisher = node.create_publisher(JointState, "/zerolab_finger_states", 10)


# create a timer to publish messages at 30Hz
def timer_callback():
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = ["joint1", "joint2", "joint3"]
    msg.position = [0.0, 0.5, 1.0]
    msg.velocity = [0.0, 0.0, 0.0]
    msg.effort = [0.0, 0.0, 0.0]
    publisher.publish(msg)
    node.get_logger().info("Published joint states passthrough")


timer = node.create_timer(1.0 / 30.0, timer_callback)
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
