#!/bin/bash
docker run \
  --rm \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/custom/location/.Xauthority:rw" \
  --net=host \
  --device=/dev:/dev \
  --volume="$HOME/docker/squid/shared_folder:/shared_folder" \
  -it \
  --privileged \
  --name squid_container \
  squid

