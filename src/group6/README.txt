# README

This README provides instructions on launching the Gazebo world and running the robot controller node for the Turtlebot3 robot in ROS 2.

## Launching Gazebo World

To launch the Gazebo world with the Turtlebot3 robot, use the following command:

ros2 launch final_project final_project.launch.py

This will start Gazebo with the Turtlebot3 robot in a maze environment.

## Running Robot Controller Node

To run the robot controller node, use the following command:

ros2 launch group6 robot_controller.launch.py

This will launch the robot controller node.

Make sure to have ROS 2 installed and sourced before executing these commands.

After making changes to the waypoint_params.yaml file, remember to rebuild and source the package
