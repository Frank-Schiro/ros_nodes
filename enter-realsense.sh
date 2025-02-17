#!/bin/bash

# Enter the container and execute commands
# docker compose exec realsense bash -c "source /opt/ros/humble/setup.bash && cd workspace/workspace && bash"
docker compose exec realsense bash -c "source /opt/ros/humble/setup.bash && cd workspace/workspace && ros2 run rviz2 rviz2 -d hand_marker_array.rviz && bash"