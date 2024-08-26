# RDJ-2024 Docker Image

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

1. Pulls the latest version of the RDJ-2024 Docker image.
2. Checks if you're running on a Raspberry Pi or a PC.
3. Checks if a container named `rdj-2024` already exists.
4. If the container exists, it starts the existing container.
5. If the container doesn't exist, it creates and runs a new container with the appropriate display forwarding setup (X11 or Wayland).

### Features

- **Automatic Platform Detection**: Automatically detects whether you're on a Raspberry Pi or PC and uses the appropriate Docker image and settings.
- **Display Forwarding**: Sets up X11 or Wayland display forwarding, allowing GUI applications to run inside the Docker container and display on the host system.
- **Persistent Containers**: Creates a named container that persists between runs, allowing you to maintain your work environment.
- **Latest Image Pull**: Always attempts to pull the latest version of the Docker image before running, ensuring you have the most up-to-date environment.

## Notes

- The script uses the image `codewithlennylen/rdj-2024:latest`. You can view it on Docker Hub by clicking [here.](https://hub.docker.com/r/codewithlennylen/rdj-2024)
- If you need to use a different image or tag, modify the `IMAGE_NAME` variable at the top of the script.
- The script creates a container named `rdj-2024`. If you need to use a different name, modify the `CONTAINER_NAME` variable.
- To use a new version of the image with an existing container, you need to manually remove the old container first:
  ```bash
  docker rm rdj-2024
  ```
  Then run the script again to create a new container with the updated image.

## Troubleshooting

- If you encounter permission issues, ensure your user is part of the Docker group:
  ```bash
  sudo usermod -aG docker $USER
  ```
  Log out and log back in for the changes to take effect.
- For X11 display issues on PC, try running `xhost +local:docker` before running the script.
- If the script fails to detect your display server correctly, you can manually modify the script to use the correct method (X11 or Wayland).
