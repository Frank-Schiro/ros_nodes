#!/bin/bash

set -e


echo "=== Starting entrypoint.sh ==="
echo "Current directory:"
pwd
echo "Directory contents:"
ls -la

echo "Python path:"
echo $PYTHONPATH
echo "Test directory contents:"
ls -la /app/tests


source /opt/ros/humble/setup.bash
source /app/vision_interfaces/install/setup.bash

echo "=== About to execute command: $@ ==="
exec "$@"