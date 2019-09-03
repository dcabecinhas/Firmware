#!/usr/bin/env bash
if [ "$1" == "" ]; then
  echo "Pass launchfile as argument."
  echo "Example: $0 multi_uav_single_urdf_mavros_sitl_4.launch"
  exit 1
fi

# export PX4_SIM_SPEED_FACTOR=0.5
DONT_RUN=1 make px4_sitl_default gazebo
if [ $? -ne 0 ]
then
  echo "Error running 'make px4_sitl_default gazebo'" >&2
  exit 1
fi

source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 $1
