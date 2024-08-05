# ROS2 Autonomous Drone Navigation and Object Detection

## Overview

This project demonstrates autonomous drone navigation using ROS2, C++, and Gazebo. It implements visual ORB SLAM for SLAM, YOLO for object detection, and PID control for navigation. The drone is designed to fly over a predefined environment, detect and record the locations of fruits, and navigate to a specified goal location.

## Prerequisites

- ROS2 (Foxy or later)
- Gazebo (compatible with ROS2)
- C++
- Pre-trained models for ORB SLAM and YOLO
- Custom Gazebo world with fruit models

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

4. **Download or create the Gazebo world**

   Save your custom Gazebo world file (e.g., `orchard.world`) in the `worlds/` directory.

   Example `orchard.world` file location: `ros2_autonomous_drone/worlds/orchard.world`

## Usage

1. **Launch the Gazebo simulation with ROS2**

    ```bash
    ros2 launch launch/drone_launch.py
    ```

2. **Monitor the drone’s progress**

   The drone will navigate through the environment, detect fruits, and record their locations.

## Directory Structure

- `src/`: Contains the ROS2 packages for ORB SLAM, YOLO, and PID controller.
- `launch/`: Contains the ROS2 launch file (`drone_launch.py`) to start the simulation and nodes.
- `worlds/`: Contains custom Gazebo world files (e.g., `orchard.world`).

## License

This project is licensed under the Apache License - see the [LICENSE](LICENSE) file for details.

## Note

Please note that some parts of the code are not provided due to plagiarism concerns. Ensure to replace placeholders like `your_email@example.com` and `yourusername` with your actual information.
