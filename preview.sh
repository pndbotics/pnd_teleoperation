source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765