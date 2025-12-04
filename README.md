# Unitree GO1 Simulation with ROS 2 jazzy Gazebo sim

This repository contains a simulation environment for the Unitree GO1 robot in Gazebo Sim and ROS 2, along with an interface for navigation. The functionality has been tested with ROS Jazzy on Ubuntu 24.04

## Dependencies:
- lcm (Needs to be built from source, instructions [here](https://lcm-proj.github.io/lcm/))
- [navigation2](https://github.com/ros-navigation/navigation2)
- [ros2_control](https://github.com/ros-controls/ros2_control)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- [gazebo_plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins)

## Testing

After placing all the packages in a ROS 2 workspace and building it successfully, run the following in order:


1. *(window 1)* `ros2 launch go1_gazebo spawn_go1_gz.launch.py`: This will load the simulation and initialize controllers.

2. *(window 2)* `ros2 run unitree_guide2 junior_ctrl`: This activates the interface and state machine. **Run this once the last controller plugin (*RL_calf_controller*) has loaded successfully**
    1. Press 2 to switch the robot to standing mode (fixed_stand)
    2. Press 5 to switch to move_base mode (robot accepts velocity commands in this mode)


## Acknowledgements

This project builds on the following open-source packages:

- [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)  
  Used as the starting point for the robot description, meshes and simulation setup.

- [Atharva-05/unitree_ros2_sim](https://github.com/Atharva-05/unitree_ros2_sim)  
  Used as a reference and starting point for the ROS 2 simulation packages for the Unitree Go1. 
  Several packages and launch files in this repository were adapted and extended for our use.


