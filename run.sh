#!/bin/bash

source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix mobile_robot_ai)/share/mobile_robot_ai/worlds/

ros2 launch mobile_robot_ai simulation_launch.py 