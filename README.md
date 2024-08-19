# spaceros_gz_demos

## Docker setup
If you don't have docker, install it with `sudo apt install docker.io`. If you get a permission denied error with the Docker daemon, you will have to add yourself to the docker user group with `sudo usermod -aG docker $USER`, then run `newgrp docker`. You may have also to run `xhost +local:docker` on your first time. You will need to log out and log back in for these changes to take effect on your system.

Build the docker image locally with

`./scripts/docker_build.sh`

Then start the container with

`./scripts/docker_start.sh`

Once in the docker container, you can run `ros2` commands as well as use gui tools like `rviz2` and `gz sim`.

Once you have a running container, to get another shell, run 

`./scripts/docker_shell.sh`

### Running inside of the docker
1. Once you are in the docker container, run `colcon build` (you should be in the `~/spaceros/ws` directory)

2. Run `source install/setup.bash`

3. Start the demo with `ros2 launch moon.launch.xml`, `mars.launch.xml`, or `enceladus.launch.xml`