source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch bringup retarget_adam_sp.launch.py visual:=rviz2
