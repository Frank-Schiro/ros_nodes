# Overview

Trying to get realsense-ros running in the d435i container

https://github.com/IntelRealSense/realsense-ros



# Development Plan

https://chatgpt.com/c/67a831dc-467c-8012-a978-f385192a854a

monorepo + docker-compose in beginning
transition to kubernetes "ros-deploymet" repo to manage all services in kubes
transition services to their own repos

## Tip

### Debug docker

Run with: `docker compose build hand_tracking --progress=plain --no-cache`

```yml
# Debug commands 
RUN echo "=== Debugging file locations ==="
RUN ls -la src/vision_interfaces/
RUN echo "=== MSG directory contents ==="
RUN ls -la src/vision_interfaces/msg/
RUN echo "=== MSG file contents ==="
RUN cat src/vision_interfaces/msg/HandLandmarkArray.msg || echo "File not found!"
RUN echo "=== End debug ==="
```

# Quick Start

```bash
xhost +local:root
docker compose up

IF IT FAILS TRY ACTIVATING peakInt!
'mdlbase'
```

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic echo /hand_detection_3d
```

If you are having trouble with no frame data
```bash
[WARN] [1738958128.649422132] [camera.camera]: XXX Hardware Notification:Frames didn't arrived within 5 seconds,1.73896e+12,Warn,Frames Timeout
```

Try:
1. `docker compose down`
2. `docker compose ps` + make sure doesn't show the container
    * To force container offline: `sudo systemctl restart docker`
3. Unplug and replug in camera


## Supported camera modes

Looking at the supported modes:

(found with these commands)
```bash
# Check current depth profile
ros2 param get /camera/camera depth_module.depth_profile
# Check current color profile
ros2 param get /camera/camera rgb_camera.color_profile
rs-enumerate-devices
```

For Depth (Z16 format):

1280x720 @ 30fps
848x480 @ up to 90fps
640x480 @ up to 90fps


For Color (RGB8 format):

1920x1080 @ 30fps
1280x720 @ 30fps
848x480 @ up to 60fps




# Quick Stop

```bash
docker compose down
# If it doesn't work you can do this and wait a minute for it to work:
sudo systemctl restart docker
```
# Install

We need the camera kernals installed on the host for the container to be able to access the camera. 
dpkg package installs kernals

udev rules on the host must be set up correctly

## Install the kernal and everything on the host to run camera

```bash
# Step 1 - install librealsense2
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

# Install the DKMS dynamic kernal module support - this is the kernal support for the camera on the host
sudo apt-get install librealsense2-dkms

# Step 2 - install librealsense2-utils
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo jammy main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt-get update
sudo apt-get install librealsense2-utils

# Step 3 - install developer packages
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

* Test locally with `realsense-viewer`W

## Allow X11 forwarding on your host machine:

```bash
xhost +local:root
```

To make this change permanent: 

```bash
echo "xhost +local:root" >> ~/.bashrc
```

What Does xhost +local:root Do?

    It grants access to the X server (the graphical display system) for processes running as root on the local machine.
    By default, X11 restricts which users can connect to the display. Since the Docker container runs as root, it needs explicit permission to use X11.
    Without this, GUI applications like realsense-viewer running inside the container won't be able to display their interface on your host's screen.

Security Considerations

    xhost +local:root grants any root process on the local machine access to your X server, which can be a security risk.
    To restrict access to just your container, you can use:

```bash
xhost +si:localuser:$USER
```

## Setting Up UDEV Rules for RealSense Camera Permissions

To allow a Docker container (or non-root user) to access the RealSense camera, you need to configure UDEV rules on the host system.

1. Create the UDEV Rules File

Run the following command to add the necessary permissions:

```bash
echo -e 'SUBSYSTEM=="usb", ATTR{idVendor}=="8086", MODE="0666"\nSUBSYSTEM=="usb", ATTR{idVendor}=="8087", MODE="0666"' | sudo tee /etc/udev/rules.d/99-realsense.rules
```

2. Reload UDEV Rules

Apply the changes by running:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo service udev restart
```

3. Verify device permissions:

```bash
ls -l /dev/bus/usb/$(lsusb | grep '8086' | awk '{print $2}')/
total 0
crw-rw-rw- 1 root root    189, 384 Feb  7 11:10 001
crw-rw-rw- 1 root root    189, 385 Feb  7 11:10 002
crw-rw-rw- 1 root plugdev 189, 395 Feb  7 11:11 012
```

Notice all devices in the list have full read + write `crw-rw-rw-`

If the permissions are still restricted, try manually setting them:
```bash
sudo chmod 666 /dev/bus/usb/$(lsusb | grep '8086' | awk '{print $2}')/*
```





# To Test

docker exec -it d435i_docker_ros-realsense-1 bash
docker exec -it d435i_docker_ros-topic_reader-1 bash

source /opt/ros/humble/setup.bash

ros2 topic list

## Testing with RVIZ

* To view the camera feed, you can use RViz2:
```bash
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2

cd workspace/workspace
ros2 run rviz2 rviz2 -d d435i.rviz
```

may need to source if docker not updated: source /opt/ros/humble/setup.bash
## Load RVIZ manually
* In RViz2:
    1. setup point cloud
        - [cntrl] + N
        - add "PointCloud2"
        - In the PointCloud2 settings:
            Set "Topic" to /camera/camera/depth/color/points
            Set "Fixed Frame" (at the top of RViz2) to camera_link
            Adjust "Size (m)" (point size) to 0.01

    2. Add rgb display
        - [cntrl] + N
        - select "Image"
        - Set "topic" to /camera/camera/color/image_raw
    3. Add depth display
        - [cntrl] + N
        - select "Image"
        - Set "topic" to /camera/camera/color/image_raw

## Save the rviz to your workspace!

```bash
# Inside container
cd /workspace
rviz2 -d config.rviz  # Save your configuration as config.rviz
```

## Testing with ROS



```bash
# Enter container
docker exec -it 7d4b5b780d1e bash
source /opt/ros/humble/setup.bash

# Check node is running
ros2 node list
# /camera/camera

# list the topics
ros2 topic list
# /camera/camera/color/camera_info
# /camera/camera/color/image_raw
# /camera/camera/color/metadata
# /camera/camera/depth/camera_info
# /camera/camera/depth/image_rect_raw
# /camera/camera/depth/metadata
# /camera/camera/extrinsics/depth_to_color
# /parameter_events
# /rosout
# /tf_static
```

* To check specific topic data

```bash
# View color camera info
source /opt/ros/humble/setup.bash
ros2 topic echo /camera/camera/color/camera_info

# View depth camera info
ros2 topic echo /camera/camera/depth/camera_info
```




To view all params
```bash
ros2 param list /camera/camera
  accel_fps
  accel_info_qos
  accel_qos
  align_depth.enable
  align_depth.frames_queue_size
  angular_velocity_cov
  base_frame_id
  camera.color.image_raw.enable_pub_plugins
  camera.depth.image_rect_raw.enable_pub_plugins
  camera_name
  clip_distance
  color_info_qos
  color_qos
  colorizer.color_scheme
  colorizer.enable
  colorizer.frames_queue_size
  colorizer.histogram_equalization_enabled
  colorizer.max_distance
  colorizer.min_distance
  colorizer.stream_filter
  colorizer.stream_format_filter
  colorizer.stream_index_filter
  colorizer.visual_preset
  decimation_filter.enable
  decimation_filter.filter_magnitude
  decimation_filter.frames_queue_size
  decimation_filter.stream_filter
  decimation_filter.stream_format_filter
  decimation_filter.stream_index_filter
  depth_info_qos
  depth_module.auto_exposure_limit
  depth_module.auto_exposure_limit_toggle
  depth_module.auto_exposure_roi.bottom
  depth_module.auto_exposure_roi.left
  depth_module.auto_exposure_roi.right
  depth_module.auto_exposure_roi.top
  depth_module.auto_gain_limit
  depth_module.auto_gain_limit_toggle
  depth_module.depth_format
  depth_module.depth_profile
  depth_module.emitter_always_on
  depth_module.emitter_enabled
  depth_module.emitter_on_off
  depth_module.enable_auto_exposure
  depth_module.error_polling_enabled
  depth_module.exposure
  depth_module.frames_queue_size
  depth_module.gain
  depth_module.global_time_enabled
  depth_module.hdr_enabled
  depth_module.infra1_format
  depth_module.infra2_format
  depth_module.infra_profile
  depth_module.inter_cam_sync_mode
  depth_module.laser_power
  depth_module.output_trigger_enabled
  depth_module.sequence_id
  depth_module.sequence_name
  depth_module.sequence_size
  depth_module.visual_preset
  depth_qos
  device_type
  diagnostics_period
  disparity_filter.enable
  disparity_to_depth.enable
  enable_accel
  enable_color
  enable_depth
  enable_gyro
  enable_infra1
  enable_infra2
  enable_rgbd
  enable_sync
  filter_by_sequence_id.enable
  filter_by_sequence_id.frames_queue_size
  filter_by_sequence_id.sequence_id
  gyro_fps
  gyro_info_qos
  gyro_qos
  hdr_merge.enable
  hdr_merge.frames_queue_size
  hold_back_imu_for_frames
  hole_filling_filter.enable
  hole_filling_filter.frames_queue_size
  hole_filling_filter.holes_fill
  hole_filling_filter.stream_filter
  hole_filling_filter.stream_format_filter
  hole_filling_filter.stream_index_filter
  infra1_info_qos
  infra1_qos
  infra2_info_qos
  infra2_qos
  initial_reset
  json_file_path
  linear_accel_cov
  motion_module.enable_motion_correction
  motion_module.frames_queue_size
  motion_module.global_time_enabled
  motion_module.gyro_sensitivity
  pointcloud.allow_no_texture_points
  pointcloud.enable
  pointcloud.filter_magnitude
  pointcloud.frames_queue_size
  pointcloud.ordered_pc
  pointcloud.pointcloud_qos
  pointcloud.stream_filter
  pointcloud.stream_format_filter
  pointcloud.stream_index_filter
  publish_tf
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  reconnect_timeout
  rgb_camera.auto_exposure_priority
  rgb_camera.auto_exposure_roi.bottom
  rgb_camera.auto_exposure_roi.left
  rgb_camera.auto_exposure_roi.right
  rgb_camera.auto_exposure_roi.top
  rgb_camera.backlight_compensation
  rgb_camera.brightness
  rgb_camera.color_format
  rgb_camera.color_profile
  rgb_camera.contrast
  rgb_camera.enable_auto_exposure
  rgb_camera.enable_auto_white_balance
  rgb_camera.exposure
  rgb_camera.frames_queue_size
  rgb_camera.gain
  rgb_camera.gamma
  rgb_camera.global_time_enabled
  rgb_camera.hue
  rgb_camera.power_line_frequency
  rgb_camera.saturation
  rgb_camera.sharpness
  rgb_camera.white_balance
  rosbag_filename
  serial_no
  spatial_filter.enable
  spatial_filter.filter_magnitude
  spatial_filter.filter_smooth_alpha
  spatial_filter.filter_smooth_delta
  spatial_filter.frames_queue_size
  spatial_filter.holes_fill
  spatial_filter.stream_filter
  spatial_filter.stream_format_filter
  spatial_filter.stream_index_filter
  temporal_filter.enable
  temporal_filter.filter_smooth_alpha
  temporal_filter.filter_smooth_delta
  temporal_filter.frames_queue_size
  temporal_filter.holes_fill
  temporal_filter.stream_filter
  temporal_filter.stream_format_filter
  temporal_filter.stream_index_filter
  tf_publish_rate
  unite_imu_method
  usb_port_id
  use_sim_time
  wait_for_device_timeout
```


# 2/8/25

How should we go about abstracting the sensor?
I want to design this. 
Why?

I want to be able to spin up multiple sensors and have a standardized interface for them all. 

## Example

Two depth sensors have different manufacturers and SDKs. 
I want to control these sensors using a port-adapter model so that applications can use a standardized port with both sensors.
That way, my applications don't need to know the internals of each sensor. 

I am working in a ROS environment. 
I don't want my code to be tied to the ROS environment. 

So my application will:

1. Initiate a publish-service 
    - Input a publish-port
        - ROS adapater
        - AWS-SNS adapter
        - port-methods:
            - publish image stream
            - publish xxx
                - xxx will have validation and be standardized like a ROS topic? 

* realsense_d435i_node is a ROS adapter which publishes standardized ROS topics.
It is automatic, a docker container and an external library handle everything. 
How to make this integrate to the publish-service model I am invisioning?

    - what is this standard? How to study it? 


2. Initiate a subscribe-service 
    - 



## Questions

The "standardized" topics published in realsense_d435i_node are outlined somewhere, in some ROS standard documentation maybe.
We should follow this same standar for all published tasks. 

Our AWS-SNS adapter would then have to publish the same topics for a given sensor. 

This topic space needs to be mapped with ports and entities. 



How do we enforce this port on realsense_d435i_node? Lets say the port has one extra method not included in realsense_d435i_node. 
Then I have to add a new node with some python code. 

But if I don't add that necissary code, then how do we validate the port in our framework? 




hand_tracking_node (Application Layer)
│── core/
│   ├── hand_tracker_port.py  (Abstract hand tracking logic)
│   ├── tracking_service.py  (Glues the tracking logic to the display service)
│   ├── display_service.py  (send video to user)
│   ├── display_port.py 
│── adapters/
│   ├── mediapipe_hand_tracker_adapter.py  (Uses MediaPipe to implement HandTrackerPort)
│   ├── opencv_display_adapter.py  (Uses OpenCV to display images)
│── main.py  (ROS Node: Initializes tracking_service & display_service with adapters & handles pub-sub. Contains a main callback function)