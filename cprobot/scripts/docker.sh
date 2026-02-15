#!/usr/bin/env bash
# bash scripts/docker.sh
set -e

IMAGE_NAME="cprobot"
HOST_WS="${HOST_WS:-$HOME/projects/cprobot}"
CONTAINER_WS="/ws"

docker run -it --rm \
  --net=host \
  -v "${HOST_WS}:${CONTAINER_WS}" \
  -w "${CONTAINER_WS}" \
  -e HOST_UID="$(id -u)" \
  -e HOST_GID="$(id -g)" \
  "${IMAGE_NAME}" \
  bash
