# 4-DOF Service Robotic Manipulator for Automated Beverage Preparation

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-orange)](https://www.mathworks.com/products/matlab.html)
[![Arduino](https://img.shields.io/badge/Arduino-Uno-teal)](https://www.arduino.cc/)

A desktop-scale **4-DOF service robotic manipulator** designed for automated beverage preparation. The system covers the full robotics development pipeline — from DH-based kinematic modeling through dual-platform simulation (Gazebo + Simscape) to physical hardware execution on a 3D-printed arm.

The robot performs a complete **tea-preparation demo**: sugar transport, spoon manipulation, and stirring — demonstrating coordinated multi-step service tasks using open-loop trajectory control.

---

## Overview

The manipulator has 5 revolute joints (Joint 4 fixed → 4 active DOF), constructed from **3D-printed PLA** and actuated by **MG996R** (high-torque) + **SG90** (micro) servo motors. Control is handled by an Arduino Uno communicating with a PC-based trajectory planner over serial.

### DH Parameters

| Link | θᵢ | dᵢ (m) | aᵢ (m) | αᵢ (rad) |
|------|-----|---------|---------|----------|
| 1 | q₁ + π/2 | 0.04355 | 0 | π/2 |
| 2 | q₂ | 0 | 0.140 | π |
| 3 | q₃ | 0 | 0.133 | π |
| 4 | q₄ | 0 | 0.109 | 0 |

### Key Results

| Metric | Value |
|--------|-------|
| FK roundtrip positional error | 0.000 m |
| IK roundtrip velocity error | ~10⁻¹⁵ (machine precision) |
| Zero-config end-effector height | 0.4255 m |
| Trajectory sampling period | 0.1 s |
| IK damping factor (λ) | 0.05 |

---

## Repository Structure

```
├── simulation/
│   ├── ros/src/                        # ROS/Gazebo simulation
│   │   └── my_robot_gazebo/            # ROS package
│   │       ├── urdf/robot.urdf         # Robot description
│   │       ├── launch/                 # Gazebo launch files
│   │       ├── meshes/                 # STL mesh files
│   │       ├── config/                 # Controller configs
│   │       ├── scripts/                # Python nodes (FK, IK, trajectory)
│   │       ├── models/                 # Gazebo world models
│   │       └── worlds/                 # Gazebo world files
│   │
│   └── matlab/                         # MATLAB/Simulink
│       ├── kinematics/                 # FK, IK, Jacobian functions
│       ├── trajectory/                 # Trajectory planners + CSV data
│       ├── simscape/                   # Simulink Simscape Multibody model
│       └── tests/                      # Validation scripts
│
├── hardware/
│   ├── arduino/                        # Arduino servo control
│   │   ├── receiver/receiver.ino       # Arduino firmware
│   │   ├── send_trajectory.py          # PC → Arduino trajectory sender
│   │   └── trajectories/             # Pre-computed CSV joint angles
│   │
│   └── cad/                            # SolidWorks CAD exports
│       ├── 5_DOF_Robot_Assembly.SLDASM # Full assembly
│       └── *.STEP                      # Individual part files
│
├── milestones/                         # Course milestone progression artifacts
│   ├── milestone-02/                   # Early FK + ROS control scripts
│   ├── milestone-03/                   # IK/Jacobian validation scripts
│   ├── milestone-04/                   # Trajectory planning experiments
│   └── README.md
│
├── .gitignore
├── LICENSE
└── README.md
```

---

## Getting Started

### Prerequisites

| Tool | Version | Purpose |
|------|---------|---------|
| ROS Noetic | Ubuntu 20.04 | Gazebo simulation |
| MATLAB | R2023b+ | Kinematics + Simscape |
| Python | 3.8+ | ROS nodes + Arduino comm |
| Arduino IDE | 2.0+ | Firmware upload |

### Python Environment

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### ROS / Gazebo

```bash
mkdir -p ~/catkin_ws/src
ln -s /path/to/this/repo/simulation/ros/src/my_robot_gazebo ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# Launch robot in Gazebo
roslaunch my_robot_gazebo spawn_gazebo.launch

# Run trajectory execution (separate terminal)
rosrun my_robot_gazebo trajectory_node.py
```

### MATLAB / Simulink

```matlab
addpath('simulation/matlab/kinematics');
addpath('simulation/matlab/trajectory');
addpath('simulation/matlab/tests');

test_forward_position     % FK validation
test_inverse_kinematics   % IK validation
test_velocity_kinematics  % Jacobian validation

% Open Simscape model
open('simulation/matlab/simscape/x5_DOF_Robot_Assembly.slx')
```

### Arduino Hardware

1. Upload `hardware/arduino/receiver/receiver.ino` to the Arduino Uno
2. Connect servos to PWM pins (Base→3, Shoulder→5, Elbow→6, Wrist→9, Gripper→10)
3. Run: `python hardware/arduino/send_trajectory.py`

See [`hardware/arduino/README.md`](hardware/arduino/README.md) for detailed setup, calibration, and troubleshooting.

---

## Authors

| Name | Affiliation |
|------|-------------|
| **Andrew Abdelmalak** | Mechatronics Engineering, GUC |
| **Daniel Boules** | Mechatronics Engineering, GUC |
| **David Girgis** | Mechatronics Engineering, GUC |
| **Kirolous Kirolous** | Mechatronics Engineering, GUC |
| **Samir Yacoub** | Mechatronics Engineering, GUC |
| **Youssef Salama** | Mechatronics Engineering, GUC |

## Report

The full project report is available in [`docs/4DOF_Service_Manipulator.pdf`](docs/4DOF_Service_Manipulator.pdf).

## License

MIT — see [LICENSE](LICENSE).