#!/bin/bash
echo "Starting docker container..."
docker run -it \
  --network host \
  -e DISPLAY \
  -e TERM \
  -e QT_X11_NO_MITSHM=1 \
  --device /dev/dri \
  --mount type=bind,source="$(pwd)",target=/home/spaceros-user/spaceros/ws/src/spaceros_gz_sim \
  spaceros_gz_sim:latest
