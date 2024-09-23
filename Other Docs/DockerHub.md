

# rdj-2024 Robotics Development Environment

## Introduction

The rdj-2024 Docker image is a comprehensive, ready-to-use container for robotics enthusiasts, researchers, and developers working with ROS2 (Robot Operating System 2) and the Navigation Stack. 
This image was built to provide a consistent, easily deployable environment for ROS2 development across different hardware platforms, specifically targeting Raspberry Pi (ARM64) and PC (AMD64) architectures.

## Why This Image Was Built

1. **Consistency**: Ensures a uniform development environment across different machines and operating systems.
2. **Ease of Use**: Eliminates the complexities of ROS2 installation and setup, allowing users to start developing immediately.
3. **Cross-Platform Compatibility**: Supports both ARM64 (Raspberry Pi) and AMD64 (PC) architectures with a single image pull command.
4. **Educational Purpose**: Provides a complete toolkit for learning and experimenting with ROS2 and robotics concepts.

## What It Contains

This Docker image includes:

- ROS2 Humble distribution
- Essential ROS2 tools and dependencies:
  - demo_nodes_cpp and demo_nodes_py
  - rviz2
  - rqt
  - joint_state_publisher_gui
  - ros-gz (Gazebo integration)
- ROS2 build tools:
  - colcon
  - rmw_cyclonedds_cpp
- Navigation Stack:
  - navigation2
  - nav2_bringup
- Turtlebot3 packages for navigation demos
- Linux tools: nano, gedit
- X11 apps and Mesa utilities for GUI support
- Pre-configured environment variables for ROS2 and Gazebo

## Installation Instructions

You can find the latest version of this image's Documentation & installation instructions on GitHub:
[roboticsdojo/rdj-2024-docker](https://github.com/roboticsdojo/rdj-2024-docker)


