# spaceros_gz_sim

## Docker setup
If you don't have docker, install it with `sudo apt install docker.io`. If you get a permission denied error with the Docker daemon, you will have to add yourself to the docker user group with `sudo usermod -aG docker $USER`, then run `newgrp docker`. You will need to log out and log back in for these changes to take effect on your system.

Build the docker image locally with

`docker build . --tag "spaceros_gz_sim"`

Then run the image with

`./scripts/docker_run.sh`

You may have to run `xhost +local:docker` on your first time. 

Once in the docker container, you can run `ros2` commands as well as use gui tools like `rviz2` and `gz sim`.
