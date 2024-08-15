# spaceros_gz_sim

## Docker setup
Build the docker image locally with

`docker build . --tag "spaceros_gz_sim"`

Then run the image in an interactive terminal, first

`xhost +local:docker`

then

`docker run -it --network host -e DISPLAY -e QT_X11_NO_MITSHM=1 --device /dev/dri spaceros_gz_sim:latest`
