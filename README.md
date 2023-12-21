# Nav2Turtle

This ROS2 project utilizes a TurtleBot3 waffle robot for navigation through predefined battery waypoints. Ensure you have the necessary dependencies installed before running the project.

## Dependencies

1. Install TurtleBot3 ROS2 packages:
   ```bash
   sudo apt install 'ros-galactic-turtlebot3*'
   sudo apt install 'ros-galactic-turtlebot3-*'
   ```

2. Install TF2 packages:
   ```bash
   sudo apt install ros-galactic-tf2
   sudo apt install ros-galactic-tf2-ros
   sudo apt install ros-galactic-tf2-tools
   ```

3. Install Navigation2 packages:
   ```bash
   sudo apt install ros-galactic-navigation2
   sudo apt install ros-galactic-nav2-bringup
   sudo apt install ros-galactic-slam-toolbox
   ```

4. Install the lastest version of OpenCV:
   ```bash
   pip3 uninstall opencv-python
   pip3 uninstall opencv-contrib-python
   pip3 install opencv-contrib-python
   ```

6. Export TurtleBot3 model:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ```

## Launching the World

To launch the world, run the following command:
```bash
ros2 launch final_project final_project.launch.py
```

## Launching the Controller

Launch the controller to navigate through battery waypoints:
```bash
ros2 launch group6 robot_controller.launch.py
```
You can customize the waypoint order in the `waypoint_params.yaml` file in the `group6` config folder. Change `aruco_1` to `aruco_0` or as needed. <br>
Reference link: https://github.com/zeidk
