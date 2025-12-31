#!/bin/bash
# Kill all ROS and Gazebo processes
echo "Killing all ROS and Gazebo processes..."
killall -9 gzserver gzclient rosmaster rosout rviz robot_state_publisher controller_spawner spawner move_group
pkill -f ros

# Wait a bit
sleep 2

# Source the workspace
source devel/setup.bash

# Launch
echo "Launching demo_gazebo.launch..."
roslaunch fr5_moveit_config demo_gazebo.launch
