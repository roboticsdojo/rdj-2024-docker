# RDJ-2024 Docker Image (v2.0.0)

## Using the Convenience Script

This script (`start-rdj-2024.sh`) simplifies the process of pulling, creating, and running ROS2 Docker containers on both Raspberry Pi (ARM64) and PC (AMD64) platforms.

### Prerequisites

1. Docker installed on your system -> [Install Docker Instructions](https://docs.docker.com/engine/install/)
2. User added to the Docker group to run Docker commands without sudo -> [Post-Install Steps](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)
3. X11 or Wayland display server running on the host system

### Usage

1. Download the script `start-rdj-2024.sh` into your desired directory.

2. Make the script executable (Linux Terminal):
   ```bash
   chmod +x start-rdj-2024.sh
   ```

3. Run the script (Linux Terminal):
   ```bash
   ./start-rdj-2024.sh
   ```

### What the Script Does

1. Removes any existing containers named `rdj-2024` to ensure a fresh start.
2. Pulls the latest version of the RDJ-2024 Docker image.
3. Checks if you're running on a Raspberry Pi or a PC.
4. Creates and runs a new container with the appropriate display forwarding setup (X11 or Wayland).

### Features

- **Automatic Platform Detection**: Automatically detects whether you're on a Raspberry Pi or PC and uses the appropriate Docker image and settings.
- **Display Forwarding**: Sets up X11 or Wayland display forwarding, allowing GUI applications to run inside the Docker container and display on the host system.
- **Fresh Environment**: Always creates a new container, ensuring a clean and up-to-date environment for each session.
- **Latest Image Pull**: Always pulls the latest version of the Docker image before running, ensuring you have the most up-to-date environment.
- **VNC Server**: Includes a pre-configured VNC server for remote GUI access.

## New in v2.0.0

- **VNC Server Integration**: Remote GUI access with Xfce4 desktop environment and TightVNC server.
- **Enhanced Linux Toolkit**: Added essential development tools including nano, gedit, X11 apps, and Mesa utilities.
- **Updated ROS2 Base**: Latest ROS Humble distribution with improved compatibility and performance.
- **Navigation Upgrades**: Integrated Navigation2 stack and Turtlebot3 packages.
- **Gazebo Integration**: Improved support for robotics simulation with ROS-Gazebo packages.

## Notes

- The script uses the image `codewithlennylen/rdj-2024:latest`. You can view it on Docker Hub by clicking [here.](https://hub.docker.com/r/codewithlennylen/rdj-2024)
- If you need to use a different image or tag, modify the `IMAGE_NAME` variable at the top of the script.
- The script creates a container named `rdj-2024`. If you need to use a different name, modify the `CONTAINER_NAME` variable.
- Each time you run the script, it will remove any existing containers with the same name and create a new one. This ensures you always start with a fresh environment.
- To connect to the VNC server, use a VNC client and connect to `localhost:5901` (password: `rdj-2024`).

## Troubleshooting

- If you encounter permission issues, ensure your user is part of the Docker group:
  ```bash
  sudo usermod -aG docker $USER
  ```
  Log out and log back in for the changes to take effect.
- For X11 display issues on PC, try running `xhost +local:docker` before running the script.
- If the script fails to detect your display server correctly, you can manually modify the script to use the correct method (X11 or Wayland).
- If you experience issues with the VNC server, ensure no other VNC sessions are running on the same port.

## Submitting Issues

If you encounter any problems or have suggestions for improvements, we encourage you to submit an issue. Here's how to do it effectively:

1. **Go to the Issues page**: Navigate to the [Issues page](https://github.com/roboticsdojo/rdj-2024-docker/issues) of the project's GitHub repository.

2. **Click "New Issue"**: Click on the green "`New Issue`" button.

3. **Choose the issue type**: Select the appropriate issue template if provided (e.g., Bug Report, Feature Request).

4. **Fill in the template**: Provide as much detail as possible. A good issue report includes:
   - A clear and descriptive title
   - A detailed description of the problem or suggestion
   - Steps to reproduce the issue (for bugs)
   - What you expected to happen
   - What actually happened
   - Your system information (OS, Docker version, hardware architecture)
   - Any relevant logs, error messages, or screenshots

5. **Code formatting**: Use markdown code blocks for any code snippets or log outputs.

6. **Submit the issue**: Once you've filled out the template, click "`Submit new issue`".

7. **Follow up**: Keep an eye on your issue for any follow-up questions or requests for additional information from the maintainers.

Remember, the more information you provide, the easier it will be for the maintainers to understand and address your issue. Be patient and respectful – maintainers are often volunteers working in their spare time.

For more detailed guidelines on contributing, please refer to our [Contribution Guidelines](CONTRIBUTION.md).