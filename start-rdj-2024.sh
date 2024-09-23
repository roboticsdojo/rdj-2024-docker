#!/bin/bash

CONTAINER_NAME="rdj-2024"
IMAGE_NAME="codewithlennylen/rdj-2024:latest"

# Function to check if container exists
container_exists() {
    docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"
}

# Function to remove existing containers
remove_existing_containers() {
    echo "Removing existing containers named ${CONTAINER_NAME}"
    docker rm -f $(docker ps -a -q --filter name=${CONTAINER_NAME})
}

# Function to pull the latest image
pull_latest_image() {
    echo "Pulling the latest image: ${IMAGE_NAME}"
    docker pull ${IMAGE_NAME}
}

# Remove existing containers
remove_existing_containers

# Pull the latest image
pull_latest_image

# Check if the hardware is Raspberry Pi
if grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "Running on Raspberry Pi"
    # touch /tmp/.docker.xauth
    # xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
    docker run -it --name ${CONTAINER_NAME} \
        --privileged \
        --network host \
        --env DISPLAY=$DISPLAY \
        ${IMAGE_NAME}
else
    echo "Running on PC"
    if [ -n "$WAYLAND_DISPLAY" ]; then
        echo "Wayland detected, using Wayland socket"
        sudo xhost +local:docker
        docker run -it --name ${CONTAINER_NAME} \
        --privileged \
        --network host \
        --env DISPLAY=$DISPLAY \
        ${IMAGE_NAME}
    else
        echo "X11 detected, using xauth"
        # XSOCK=/tmp/.X11-unix
        # XAUTH=/tmp/.docker.xauth
        # touch $XAUTH
        # xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
        sudo xhost +local:docker
        docker run -it --name ${CONTAINER_NAME} \
        --privileged \
        --network host \
        --env DISPLAY=$DISPLAY \
        ${IMAGE_NAME}
    fi
fi

echo "Container ${CONTAINER_NAME} has been stopped. To use the image again, please rerun this script."