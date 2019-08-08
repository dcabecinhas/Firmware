#!/usr/bin/env bash
if [ "$1" == "" ]; then
  echo "Pass launchfile as argument."
  echo "Example: $0 multi_uav_single_urdf_mavros_sitl_4.launch"
  exit 1
fi
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 $1
