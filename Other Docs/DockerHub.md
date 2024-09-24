
# rdj-2024 Robotics Development Environment

## Introduction

The rdj-2024 Docker image (v2.0.0) is a comprehensive, ready-to-use container for robotics enthusiasts, researchers, and developers working with ROS2 (Robot Operating System 2) and the Navigation Stack. This image provides a consistent, easily deployable environment for ROS2 development across different hardware platforms, specifically targeting Raspberry Pi (ARM64) and PC (AMD64) architectures.

## What's New in v2.0.0

- **VNC Server Integration**: Remote GUI access with Xfce4 desktop environment and TightVNC server.
- **Enhanced Linux Toolkit**: Added essential development tools including nano, gedit, X11 apps, and Mesa utilities.
- **Updated ROS2 Base**: Latest ROS Humble distribution with improved compatibility and performance.
- **Navigation Upgrades**: Integrated Navigation2 stack and Turtlebot3 packages.

## Why This Image Was Built

1. **Consistency**: Ensures a uniform development environment across different machines and operating systems.
2. **Ease of Use**: Eliminates the complexities of ROS2 installation and setup, allowing users to start developing immediately.
3. **Cross-Platform Compatibility**: Supports both ARM64 (Raspberry Pi) and AMD64 (PC) architectures with a single image pull command.
4. **Educational Purpose**: Provides a complete toolkit for learning and experimenting with ROS2 and robotics concepts.
5. **Remote Development**: New VNC server integration allows for seamless remote work and headless setups.

## What It Contains

This Docker image includes:

- ROS2 Humble distribution (latest version)
- VNC Server with Xfce4 desktop environment for remote GUI access
- Essential ROS2 tools and dependencies:
  - demo_nodes_cpp and demo_nodes_py
  - rviz2
  - rqt
  - joint_state_publisher_gui
  - ros-gz (Gazebo integration)
- ROS2 build tools:
  - colcon
  - rmw_cyclonedds_cpp (default RMW implementation)
- Navigation Stack:
  - navigation2
  - nav2_bringup
- Turtlebot3 packages for navigation demos
- Turtlesim for ROS2 demonstrations
- Enhanced Linux toolkit:
  - Text editors: nano, gedit
  - X11 applications support
  - Mesa utilities and libraries for improved graphics handling
- Pre-configured environment variables for ROS2, Gazebo, and Turtlebot3

## Installation Instructions

You can find the latest version of this image's documentation & installation instructions on GitHub:
[roboticsdojo/rdj-2024-docker](https://github.com/roboticsdojo/rdj-2024-docker)

## Usage

To pull the latest image:

```
docker pull codewithlennylen/rdj-2024:latest
```

After running the container, you can connect to the VNC server using a VNC client at `<host-ip>:5901` with the password `rdj-2024`.

## Support

For any issues or suggestions, please open an [issue](https://github.com/roboticsdojo/rdj-2024-docker/issues) in the GitHub repository.
