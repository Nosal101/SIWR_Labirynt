# My Robot Project

This project focuses on predicting the position of a robot using data from Lidar, ODOM, and IMU sensors. 

Below are the instructions on how to run the different components of this project.

## Running Instructions

| Command | Description |
|---------|-------------|
| `ros2 launch my_robot_bringup my_robot_gazebo.launch.xml` | Launch the robot in Gazebo simulation. |
| `ros2 launch my_robot_cartographer amcl.launch.py` | Launch the AMCL localization using Cartographer. |
| `ros2 run robot_control control` | Run the robot control node. |
| `ros2 run my_robot_pos my_robot_pos` | Run the position prediction node of the robot. |

## Example

Here is an example of the expected result after running the above commands:

![Robot Simulation](photos/image1.png)
