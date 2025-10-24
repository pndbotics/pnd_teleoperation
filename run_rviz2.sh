#!/bin/bash

adam_type=$1
if [ "$adam_type" == "" ]; then
    adam_type="adam_sp"
fi
if [ "$adam_type" != "adam_sp" ] && [ "$adam_type" != "adam_u" ] && [ "$adam_type" != "adam_pro" ]; then
	echo $adam_type
    echo "adam_type mast be adam_sp|adam_u|adam_pro. example: ./run.sh adam_sp"
	exit 1
fi

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

if [ "$adam_type" == "adam_sp" ]; then
    echo "run adam_sp"
    ros2 launch bringup retarget_adam_sp.launch.py visual:=rviz2
elif [ "$adam_type" == "adam_u" ]; then
    echo "run adam_u"
    ros2 launch bringup retarget_adam_u.launch.py visual:=rviz2
elif [ "$adam_type" == "adam_pro" ]; then
    echo "run adam_pro"
    ros2 launch bringup retarget_adam_pro.launch.py visual:=rviz2
fi
