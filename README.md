# Hunter 2.0 Autonomous Navigation (ROS2)

## Overview
This project enables autonomous navigation of the Hunter 2.0 mobile platform in a CoppeliaSim environment.

It includes:
- `hunter_nav`: navigation and control (Nonlinear Dynamical Systems-based)
- `obstacle_detection_ros2`: obstacle processing
- `pointcloud_to_laserscan`: LiDAR data conversion

---

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- CoppeliaSim Edu 4.10.0

---

## Setup

Clone the repository:

```bash
git clone https://github.com/dhaalmeida/hunter2-navigation.git
cd hunter2-navigation/ros2_ws
```

Build the workspace:

```bash
colcon build
```

Source ROS2 and the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Run Simulation

Start CoppeliaSim:

```bash
cd ~/VREP
./coppeliaSim
```

Load scene:

File → Open Scene → NavegacaoHunter.ttt

---

## Run System

Launch all ROS2 nodes with a single command:

```bash
ros2 launch hunter_nav hunter_sim.launch.py
```

---

## Notes

- Make sure the workspace is sourced before running the launch file
- If you rebuild, re-source:

```bash
source install/setup.bash
```
