# spaceros_gz_demos

This is a ROS 2 package demonstrating how to use Gazebo for robotic simulations in the Space ROS environment. The simulated worlds include a submersible robot on Enceladus, the Perseverance rover and Ingenuity helicopter on Mars, and swarm of construction rovers on the Moon. Gravity of each world is set to their actual values, and the Moon uses the Selenographic Coordinate System (SCS).

## Docker setup

If you don't have docker, install it with `sudo apt install docker.io`. If you get a permission denied error with the Docker daemon, you will have to add yourself to the docker user group with `sudo usermod -aG docker $USER`, then run `newgrp docker`. You may have also to run `xhost +local:docker` on your first time. You will need to log out and log back in for these changes to take effect on your system.

1. Navigate to the `docker` directory

2. Build the docker image locally with

    ```./docker_build.sh```

3. Start the container with

    ```./docker_start.sh```

4. Once you have a running container, to get another shell, run 

    ```./docker_shell.sh```

## Running code in the docker container
1. Once you are in the docker container, run `colcon build` (you should be in the `~/spaceros/ws` directory)

2. Run `source install/setup.bash`

3. Start one of the demos with `ros2 launch spaceros_gz_demos moon.launch.xml`, `mars.launch.xml`, or `enceladus.launch.xml`

## Example commands
### Moon
To move the youBot run the following command in a new sourced terminal
```bash
ros2 topic pub /model/youBot/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.5
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```