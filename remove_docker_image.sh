#!/bin/bash

# This script stops, removes containers, and deletes a Docker image by name.
# Usage:
#   ./remove_docker_image.sh <image_name>
# Example:
#   ./remove_docker_image.sh test_client
# Find containers with "docker ps -a" command

# Check if an image name was provided
if [ -z "$1" ]; then
    echo "Usage: $0 <image_name>"
    exit 1
fi

IMAGE_NAME=$1

echo "Stopping any running containers using image: $IMAGE_NAME..."
# docker ps -q --filter "ancestor=$IMAGE_NAME" | xargs -r docker stop
# docker ps -q --filter "ancestor=$IMAGE_NAME" | xargs -r docker kill
docker ps -aq --filter "ancestor=$IMAGE_NAME" | xargs -r docker rm -f

echo "Removing any containers using image: $IMAGE_NAME..."
docker ps -aq --filter "ancestor=$IMAGE_NAME" | xargs -r docker rm

echo "Deleting image: $IMAGE_NAME..."
docker images -q "$IMAGE_NAME" | xargs -r docker rmi

echo "Cleanup complete."


# docker stop $(docker ps -q)