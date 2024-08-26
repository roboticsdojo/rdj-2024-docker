
The `codewithlennylen/rdj-2024:pc-latest` Docker image is designed for roboticists and is based on the Robotics Dojo Competition 2024. It provides a robust and ready-to-use environment. 

Features include:
- **ROS2 Framework**: Pre-installed to facilitate the development of robot applications.
- **Navigation Stack**: Pre-configured for seamless integration and use.
- **Continuous Testing and Updates**: Ensuring reliability and the latest features.


---
## Instructions to Pull and Run the Docker Image

### Pull the Docker Image

To pull the Docker image, use the following command:
```sh
docker pull codewithlennylen/rdj-2024:pc-latest
```

### Enable GUI

To allow the docker container to share graphical applications with your PC, you need to run the following command:
```sh
sudo xhost +local:docker
```



### Run the Docker Image

To run the Docker image, use the following command:
```sh
docker run --name rdj-2024 -it --privileged --net=host --env DISPLAY=$DISPLAY codewithlennylen/rdj-2024:pc-latest
```

Breakdown of the command:
- **`docker run --name rdj-2024`**: Starts a container called `rdj-2024` with the `codewithlennylen/rdj-2024:pc-latest` image that you've downloaded.
- **`-it`**: Attaches the current terminal session to the running docker container
- **`--privileged`**: allows the docker container to access your computer hardware (useful for when you have sensors attached e.g. LiDAR)
- **`--net=host`**: allows the docker container to use your PC's network interface (useful for discovering ROS2 nodes in your network)
- **`--env DISPLAY=$DISPLAY`**: forwards the display of the docker container to your PC, thus allowing you to view graphical applications such as **`rqt_graph`**

### Working inside the container

The command above starts a docker container and attaches the current terminal session to the container. 
You might want to open up additional sessions to, for example, run multiple ROS2 nodes.

Open a new terminal session and run the command below to attach the terminal to the running container.

```sh
docker exec -it rdj-2024 bash
```
---
## Running ROS2 Commands

ROS requires that you '***source***' it before running commands such as `ros2 run ...`
At the start of each terminal session (inside the docker container), run the following command:

```sh
source /opt/ros/humble/setup.bash
```

Start RViz
```sh
rviz2
```

---
## Cleanup i.e. Removing a container and/or deleting an Image


The command below can be used at any time to show a list of all your containers and whether or not they are running:
```sh
docker ps -a
```

In a different terminal, type the following command to shut down the container.
```sh
docker stop rdj-2024
```

Next, remove the container:
```sh
docker rm rdj-2024
```

### Deleting the Image [WARNING]

You can also remove the image but this requires you to pull the image again in order to run another container.


The command below can be used at any time to show a list of all your images:
```sh
docker images
```

Remove the image:
```sh
docker rmi codewithlennylen/rdj-2024:pc-latest
```
