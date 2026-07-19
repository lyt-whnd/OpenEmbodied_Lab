# OpenEmbodied_Lab

OpenEmbodied_Lab is an open-source laboratory for robotics, embedded systems, and embodied AI.

This repository documents my learning journey and development projects, including ROS 2, Embedded Linux, STM32, RK3568, PCB design, computer vision, and intelligent robotic systems.

---

## Current Project

### ROS 2 Active Vision Gimbal

A vision tracking system based on ROS 2, OpenCV, and STM32.

Current workflow:

USB Camera
→ ROS 2 Image
→ OpenCV Target Detection
→ PD Controller
→ Serial Communication
→ STM32
→ PWM
→ Two-axis Servo Gimbal

---

## Features

- ROS 2 workspace
- USB camera image acquisition
- OpenCV target detection
- Target center offset calculation
- PD controller
- ROS 2 serial communication
- STM32 UART protocol
- PWM servo control
- Automatic two-axis target tracking

---

## Development Environment

- Ubuntu 24.04
- ROS 2 Jazzy
- OpenCV
- STM32CubeMX
- STM32 HAL
- Python
- C/C++

---

## Installation

```bash
git clone https://github.com/<your_username>/OpenEmbodied_Lab.git

cd OpenEmbodied_Lab

sudo apt update

sudo apt install -y \
ros-jazzy-desktop \
ros-jazzy-usb-cam \
ros-jazzy-cv-bridge \
python3-opencv \
python3-serial \
python3-colcon-common-extensions

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## Run

Start USB camera

```bash
ros2 run usb_cam usb_cam_node_exe
```

Start vision node

```bash
ros2 run vision_tracker color_tracker
```

Start serial node

```bash
ros2 run serial_controller serial_node
```

---

## Repository Structure

```text
OpenEmbodied_Lab/
├── src/
├── README.md
└── .gitignore
```

Generated directories are ignored by Git:

```text
build/
install/
log/
```

---

## Roadmap

### Completed

- [x] ROS 2 workspace
- [x] USB camera driver
- [x] OpenCV target detection
- [x] PD tracking controller
- [x] STM32 serial communication
- [x] PWM servo control
- [x] Two-axis vision gimbal

### In Progress

- [ ] RK3568 deployment
- [ ] ROS 2 launch files
- [ ] IMU integration
- [ ] Millimeter-wave radar
- [ ] ESP32-CAM integration
- [ ] Mobile robot platform
- [ ] ROS 2 Nav2
- [ ] Embedded AI applications

---

## Future Plans

This repository will continue to grow with projects related to:

- Embedded Linux
- STM32
- RK3568
- ROS 2
- PCB Design
- Robotics
- Computer Vision
- Embodied AI

