---

# **Unitree GO1 Simulation with ROS 2 Jazzy Gazebo**

This repository contains a simulation environment for the **Unitree GO1 robot** in **Gazebo Sim** and **ROS 2**, along with an interface for navigation.
The functionality has been tested with **ROS Jazzy** on **Ubuntu 24.04**.

---

## **Dependencies:**

* **[LCM](https://lcm-proj.github.io/lcm/)** – Needs to be built from source
* **[Navigation2](https://github.com/ros-navigation/navigation2)**
* **[ros2_control](https://github.com/ros-controls/ros2_control)**
* **[ros2_controllers](https://github.com/ros-controls/ros2_controllers)**
* **[Gazebo ROS 2 Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins)**

---

### **1. ROS 2 Jazzy, Gazebo Sim & RViz (if not installed)**

Install **ROS 2**, **RViz2**, and **Gazebo integration packages**. Skip if already installed.

```bash
sudo apt update && sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-colcon-common-extensions

sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image
```

---

### **2. Build & Runtime Dependencies**

#### **2.1 Essential system packages**

```bash
sudo apt install -y \
    build-essential cmake git python3-pip \
    libeigen3-dev libprotobuf-dev protobuf-compiler
```

#### **2.2 Navigation2**

```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

#### **2.3 ros2_control and ros2_controllers**

```bash
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-imu-sensor-broadcaster
```

#### **2.4 Gazebo ROS 2 Plugins**

```bash
sudo apt install -y ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-plugins
```

#### **2.5 ROS 2 Message Packages**

```bash
sudo apt install -y \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-nav-msgs
```

#### **2.6 go1_description dependencies**

```bash
sudo apt install -y ros-jazzy-robot-state-publisher
```

#### **2.7 LCM (build from source)**

```bash
cd ~
git clone https://github.com/lcm-proj/lcm.git
cd lcm
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

---

## **3. Build Your Workspace**

After installing dependencies:

```bash
cd ~/your_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## **Testing**

After a successful build, run the simulation and interface in **separate terminals**:

### **Terminal 1 – Launch Gazebo Simulation & Controllers**

```bash
ros2 launch go1_gazebo spawn_go1_gz.launch.py
```

This will load the simulation and initialize controllers.

### **Terminal 2 – Activate Unitree Go1 Interface & State Machine**

```bash
ros2 run unitree_guide2 junior_ctrl
```

This activates the interface and state machine.
**Run this once the last controller plugin (*RL_calf_controller*) has loaded successfully**

* Press **2** to switch the robot to standing mode (`fixed_stand`)
* Press **5** to switch to move_base mode (robot accepts velocity commands)

---

## **Acknowledgements**

This project builds upon the following open-source packages:

* **[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)**
  Used as the starting point for the robot description, meshes, and simulation setup.

* **[Atharva-05/unitree_ros2_sim](https://github.com/Atharva-05/unitree_ros2_sim)**
  Reference and starting point for ROS 2 simulation packages for Unitree Go1.
  Several packages and launch files in this repository were adapted and extended.

---

