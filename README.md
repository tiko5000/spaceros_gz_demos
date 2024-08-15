# spaceros_gz_sim

## Docker setup
Build the docker image locally with

`docker build . --tag "spaceros_gz_sim"`

Then run the image with

`./scripts/docker_run.sh`

You may have to run `xhost +local:docker` on your first time. 

Once in the docker container, you can run `ros2` commands as well as use gui tools like `rviz2` and `gz sim`.
