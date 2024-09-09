Active code page: 1252
Active code page: 1252
Active code page: 1252
#!/bin/bash

CONTAINER_NAME="rdj-2024"
IMAGE_NAME="codewithlennylen/rdj-2024:latest"

# Function to check if container exists
container_exists() {
  docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"
}

# Function to start existing container
start_container() {
  echo "Starting existing container ${CONTAINER_NAME}"
  docker start -ia ${CONTAINER_NAME}
}

# Function to pull the latest image
pull_latest_image() {
  echo "Pulling the latest image: ${IMAGE_NAME}"
  docker pull ${IMAGE_NAME}
}

# Pull the latest image
pull_latest_image

# Check if the hardware is Raspberry Pi
if grep -q "Raspberry Pi" /proc/cpuinfo; then
  echo "Running on Raspberry Pi"
  if container_exists; then
    start_container
  else
    touch /tmp/.docker.xauth
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
    docker run -it --name ${CONTAINER_NAME} --privileged \
      --network host \
      --volume /tmp/.docker.xauth:/tmp/.docker.xauth \
      --volume /tmp/.X11-unix:/tmp/.X11-unix \
      --env DISPLAY=$DISPLAY \
      --env XAUTHORITY=/tmp/.docker.xauth \
      ${IMAGE_NAME}
  fi
else
  echo "Running on PC"
  if container_exists; then
    start_container
  else
    if [ -n "$WAYLAND_DISPLAY" ]; then
      echo "Wayland detected, using Wayland socket"
      docker run -it --name ${CONTAINER_NAME} \
        --network host \
        -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
        -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
        -e XDG_RUNTIME_DIR=/tmp \
        ${IMAGE_NAME}
    else
      echo "X11 detected, using xauth"
      XSOCK=/tmp/.X11-unix
      XAUTH=/tmp/.docker.xauth
      touch $XAUTH
      xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
      xhost +local:docker
      docker run -it --name ${CONTAINER_NAME} \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v $XSOCK:$XSOCK:ro \
        -v $XAUTH:$XAUTH \
        -e XAUTHORITY=$XAUTH \
        --device /dev/dri \
        ${IMAGE_NAME}
    fi
  fi
fi

