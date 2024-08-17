#!/bin/bash
export DOCKER_ID=$(docker ps | grep spaceros_gz_sim:latest | awk '{print $1;}')
docker exec -it $DOCKER_ID bash
