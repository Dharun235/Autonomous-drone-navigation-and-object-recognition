# ROS2 Autonomous Drone Navigation and Object Detection

## Overview

This project demonstrates autonomous drone navigation using ROS2, C++, and Gazebo. It implements visual ORB SLAM for SLAM, YOLO for obstacle detection, and PID control for navigation. The drone navigates through a predefined environment, avoiding obstacles and reaching a target location.

## Prerequisites

- ROS2 (Foxy or later)
- Gazebo (compatible with ROS2)
- C++
- Pre-trained models for ORB SLAM and YOLO

## Installation

1. **Clone the repository**

    ```bash
    git clone https://github.com/yourusername/ros2_autonomous_drone.git
    cd ros2_autonomous_drone
    ```

2. **Install dependencies**

    ```bash
    sudo apt update
    sudo apt install ros-foxy-cv-bridge ros-foxy-image-transport ros-foxy-vision-msgs libopencv-dev
    ```

3. **Build the workspace**

    ```bash
    cd ~/ros2_autonomous_drone
    colcon build
    source install/setup.bash
    ```

## Usage

1. **Launch the nodes**

    ```bash
    ros2 launch launch/drone_launch.py
    ```

## Directory Structure

- `src/`: Contains the ROS2 packages for ORB SLAM, YOLO, and PID controller.
- `launch/`: Launch file for starting the drone navigation system.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
