#!/bin/bash

source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix mobile_robot_ai)/share/mobile_robot_ai/worlds/

gnome-terminal -- bash -c "echo 'Running data_subscriber...' ; ros2 run mobile_robot_ai data_subscriber"
gnome-terminal -- bash -c "echo 'Running open_ai_control...' ; ros2 run mobile_robot_ai open_ai_control.py"
ros2 launch mobile_robot_ai simulation_launch.py 
#ros2 launch mobile_robot_ai simulation_launch.py cmd_vel_remap:=/mobile_robot_ai/cmd_vel

#ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"