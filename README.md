
# See the output of code in docker compose

We have to fix the python-buffer, docker-cache, and docker log level 

* --progress=plain: Set D's log level by default D hides messages unless there is an error
* --no-cache: invalidate D's cache
* -u: Fix the python buffer

```bash
docker compose -f ./tests/docker-compose.yml build --progress=plain --no-cache
```

* In the dockerfile, use -u for unbuffered output. 

```Dockerfile
RUN echo "=========================================="
RUN echo "=== Testing pytest  ==="
RUN echo "=========================================="
RUN python3 -u -m pytest --version
```

* This should print

```bash
#17 0.296 pytest 6.2.5
```




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



[DEBUG] [1739580287.061995401] [hand_3d_tracking_node]: Received HandDetection2D message: vision_interfaces.msg.HandDetection2D(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1739580286, nanosec=966527588), frame_id='camera_color_optical_frame'), hands=[vision_interfaces.msg.Hand2D(hand_id='hand_0', handedness='RIGHT', 
landmarks=[vision_interfaces.msg.HandLandmark2D(name='WRIST', pixel_x=135, pixel_y=274), 
vision_interfaces.msg.HandLandmark2D(name='THUMB_CMC', pixel_x=217, pixel_y=183), 
vision_interfaces.msg.HandLandmark2D(name='THUMB_MCP', pixel_x=325, pixel_y=144), 
vision_interfaces.msg.HandLandmark2D(name='THUMB_IP', pixel_x=405, pixel_y=158), 
vision_interfaces.msg.HandLandmark2D(name='THUMB_TIP', pixel_x=455, pixel_y=184), 
vision_interfaces.msg.HandLandmark2D(name='INDEX_FINGER_MCP', pixel_x=381, pixel_y=144), 
vision_interfaces.msg.HandLandmark2D(name='INDEX_FINGER_PIP', pixel_x=491, pixel_y=132), 
vision_interfaces.msg.HandLandmark2D(name='INDEX_FINGER_DIP', pixel_x=543, pixel_y=135), 
vision_interfaces.msg.HandLandmark2D(name='INDEX_FINGER_TIP', pixel_x=578, pixel_y=133), 
vision_interfaces.msg.HandLandmark2D(name='MIDDLE_FINGER_MCP', pixel_x=383, pixel_y=206), 
vision_interfaces.msg.HandLandmark2D(name='MIDDLE_FINGER_PIP', pixel_x=463, pixel_y=213), 
vision_interfaces.msg.HandLandmark2D(name='MIDDLE_FINGER_DIP', pixel_x=411, pixel_y=219), 
vision_interfaces.msg.HandLandmark2D(name='MIDDLE_FINGER_TIP', pixel_x=358, pixel_y=211), 
vision_interfaces.msg.HandLandmark2D(name='RING_FINGER_MCP', pixel_x=376, pixel_y=269), 
vision_interfaces.msg.HandLandmark2D(name='RING_FINGER_PIP', pixel_x=439, pixel_y=272), 
vision_interfaces.msg.HandLandmark2D(name='RING_FINGER_DIP', pixel_x=384, pixel_y=273),
 vision_interfaces.msg.HandLandmark2D(name='RING_FINGER_TIP', pixel_x=339, pixel_y=266), 
 vision_interfaces.msg.HandLandmark2D(name='PINKY_MCP', pixel_x=361, pixel_y=328), 
 vision_interfaces.msg.HandLandmark2D(name='PINKY_PIP', pixel_x=414, pixel_y=329), 
 vision_interfaces.msg.HandLandmark2D(name='PINKY_DIP', pixel_x=376, pixel_y=319), 
 vision_interfaces.msg.HandLandmark2D(name='PINKY_TIP', pixel_x=342, pixel_y=314)
 ], tracking_confidence=0.8633601665496826)], image_width=848, image_height=480)

d435i_docker_ros-image_point_to_3d-1  | [INFO] [1739580287.064569658] [hand_3d_tracking_node]: Published HandDetection3D object to topic: 'hand_detection_3d' with 1 hands
d435i_docker_ros-image_point_to_3d-1  | [DEBUG] [1739580287.064889610] [hand_3d_tracking_node]: HandDetection3D: vision_interfaces.msg.HandDetection3D(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1739580286, nanosec=966527588), frame_id='camera_link'), 
hands=[
    vision_interfaces.msg.Hand3D(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), 
    hand_id='hand_0', 
    landmarks=[vision_interfaces.msg.HandLandmark3D(name='WRIST', position=geometry_msgs.msg.Point(x=-9.164176940917969, y=1.1500016450881958, z=13.432001113891602), orientation=geometry_msgs.msg.Quaternion(x=-0.03693656683877652, y=-0.294341521637242, z=0.0, w=0.9549862609851713)), vision_interfaces.msg.HandLandmark3D(name='THUMB_CMC', position=geometry_msgs.msg.Point(x=-0.19268544018268585, y=-0.05007766932249069, z=0.3930000066757202), orientation=geometry_msgs.msg.Quaternion(x=0.1280051777733132, y=0.9450071786359582, z=0.0, w=0.3009586463116296)), vision_interfaces.msg.HandLandmark3D(name='THUMB_MCP', position=geometry_msgs.msg.Point(x=-0.08643336594104767, y=-0.0796167179942131, z=0.36400002241134644), orientation=geometry_msgs.msg.Quaternion(x=-0.08343441117008933, y=0.9329254608501378, z=0.0, w=0.35026958693306276)), vision_interfaces.msg.HandLandmark3D(name='THUMB_IP', position=geometry_msgs.msg.Point(x=-0.009080012328922749, y=-0.033657267689704895, z=0.1810000091791153), orientation=geometry_msgs.msg.Quaternion(x=-0.2373605349616268, y=0.7186707023600716, z=2.7755575615628914e-17, w=0.6535842700157434)), 
    vision_interfaces.msg.HandLandmark3D(name='THUMB_TIP', position=geometry_msgs.msg.Point(x=0.02321065217256546, y=-0.04340380057692528, z=0.34700000286102295), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), 
    vision_interfaces.msg.HandLandmark3D(name='INDEX_FINGER_MCP', position=geometry_msgs.msg.Point(x=-0.019143424928188324, y=-0.03937090188264847, z=0.18000000715255737), orientation=geometry_msgs.msg.Quaternion(x=0.12650048395832042, y=0.9442526504711115, z=0.0, w=0.3039482844771313)), vision_interfaces.msg.HandLandmark3D(name='INDEX_FINGER_PIP', position=geometry_msgs.msg.Point(x=0.051851075142621994, y=-0.08465944230556488, z=0.34300002455711365), orientation=geometry_msgs.msg.Quaternion(x=0.1111154942498626, y=0.29694191899319333, z=-1.3016036733825414e-17, w=0.948408584778864)), 
    vision_interfaces.msg.HandLandmark3D(name='INDEX_FINGER_DIP', position=geometry_msgs.msg.Point(x=0.09224250912666321, y=-0.08105145394802094, z=0.33800002932548523), orientation=geometry_msgs.msg.Quaternion(x=-0.030755884189877524, y=0.7471718810769363, z=-0.0, w=0.663918862298437)), vision_interfaces.msg.HandLandmark3D(name='INDEX_FINGER_TIP', position=geometry_msgs.msg.Point(x=0.11887325346469879, y=-0.08190060406923294, z=0.33500000834465027), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), 
    vision_interfaces.msg.HandLandmark3D(name='MIDDLE_FINGER_MCP', position=geometry_msgs.msg.Point(x=-1.2663017511367798, y=-0.9164222478866577, z=12.455000877380371), orientation=geometry_msgs.msg.Quaternion(x=0.12027125332177657, y=0.9452927452041632, z=-0.0, w=0.3032432216699765)), vision_interfaces.msg.HandLandmark3D(name='MIDDLE_FINGER_PIP', position=geometry_msgs.msg.Point(x=0.02953832969069481, y=-0.019730867817997932, z=0.3450000286102295), orientation=geometry_msgs.msg.Quaternion(x=-0.5851349840438614, y=0.8085328096899006, z=-0.0, w=0.06238386091727152)), vision_interfaces.msg.HandLandmark3D(name='MIDDLE_FINGER_DIP', position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0306379938162663, y=-0.0823703719234673, z=-0.0, w=0.9961307319644865)), 
    vision_interfaces.msg.HandLandmark3D(name='MIDDLE_FINGER_TIP', position=geometry_msgs.msg.Point(x=-2.1517770290374756, y=-0.8310799598693848, z=13.432001113891602), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), 
    vision_interfaces.msg.HandLandmark3D(name='RING_FINGER_MCP', position=geometry_msgs.msg.Point(x=-0.021604593843221664, y=0.013525716960430145, z=0.18300001323223114), orientation=geometry_msgs.msg.Quaternion(x=0.11508793512315628, y=0.9457532929003984, z=0.0, w=0.30381816298098546)), vision_interfaces.msg.HandLandmark3D(name='RING_FINGER_PIP', position=geometry_msgs.msg.Point(x=0.012125899083912373, y=0.033344950526952744, z=0.41200003027915955), orientation=geometry_msgs.msg.Quaternion(x=0.5293733741053782, y=0.8455667653272276, z=-0.0, w=0.06914243387628757)), vision_interfaces.msg.HandLandmark3D(name='RING_FINGER_DIP', position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.9389223986574706, y=-0.3414393506010064, z=0.0, w=0.0429406469497706)), 
    vision_interfaces.msg.HandLandmark3D(name='RING_FINGER_TIP', position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), 
    vision_interfaces.msg.HandLandmark3D(name='PINKY_MCP', position=geometry_msgs.msg.Point(x=-0.03722139820456505, y=0.051524817943573, z=0.24300001561641693), orientation=geometry_msgs.msg.Quaternion(x=0.11367826475299901, y=0.9470426974997231, z=-0.0, w=0.30031213967336506)), vision_interfaces.msg.HandLandmark3D(name='PINKY_PIP', position=geometry_msgs.msg.Point(x=-0.0068957190960645676, y=0.050807446241378784, z=0.2370000183582306), orientation=geometry_msgs.msg.Quaternion(x=0.11734379035645406, y=0.2811667428076605, z=2.7755575615628914e-17, w=0.9524577143389156)), 
    vision_interfaces.msg.HandLandmark3D(name='PINKY_DIP', position=geometry_msgs.msg.Point(x=-0.029986703768372536, y=0.048505447804927826, z=0.2540000081062317), orientation=geometry_msgs.msg.Quaternion(x=0.5433340444023868, y=-0.7999605990872498, z=0.0, w=0.25465890147668824)), vision_interfaces.msg.HandLandmark3D(name='PINKY_TIP', position=geometry_msgs.msg.Point(x=-0.034985024482011795, y=0.031729161739349365, z=0.1770000010728836), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))], palm_pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))])

d435i_docker_ros-image_point_to_3d-1  | [DEBUG] [1739580287.065096831] [hand_3d_tracking_node]: starting marker topic
d435i_docker_ros-image_point_to_3d-1  | [DEBUG] [1739580287.065261732] [hand_3d_tracking_node]: self.display initiated
d435i_docker_ros-image_point_to_3d-1  | [DEBUG] [1739580287.065436843] [hand_3d_tracking_node]: points_of_interest: ['WRIST', 'INDEX_FINGER_MCP', 'INDEX_FINGER_TIP', 'MIDDLE_FINGER_MCP', 'RING_FINGER_MCP', 'PINKY_MCP']
d435i_docker_ros-image_point_to_3d-1  | [INFO] [1739580287.065592894] [hand_3d_tracking_node]: WRIST: (-9.164, 1.150, 13.432)
d435i_docker_ros-image_point_to_3d-1  | [INFO] [1739580287.065742995] [hand_3d_tracking_node]: INDEX_FINGER_MCP: (-0.019, -0.039, 0.180)
d435i_docker_ros-image_point_to_3d-1  | [INFO] [1739580287.065891396] [hand_3d_tracking_node]: INDEX_FINGER_TIP: (0.119, -0.082, 0.335)
d435i_docker_ros-image_point_to_3d-1  | [INFO] [1739580287.066048167] [hand_3d_tracking_node]: MIDDLE_FINGER_MCP: (-1.266, -0.916, 12.455)
d435i_docker_ros-image_point_to_3d-1  | [INFO] [1739580287.066196808] [hand_3d_tracking_node]: RING_FINGER_MCP: (-0.022, 0.014, 0.183)
d435i_docker_ros-image_point_to_3d-1  | [INFO] [1739580287.066342209] [hand_3d_tracking_node]: PINKY_MCP: (-0.037, 0.052, 0.243)


135/381 = 0.354330708661
9.164/0.019 = 482.315789474
Pixel Ratios:

    The pixel coordinates are in the 2D image space, and their ratios reflect the relative positions of the joints in the image.

    For example, if the wrist is at (135, 274) and the index finger MCP is at (381, 144), the ratio of their x-coordinates (135/381 ≈ 0.354) tells us that the wrist is about 35.4% of the way from the left edge of the image to the index finger MCP.

3D Ratios:

    The 3D coordinates are in the real-world space, and their ratios should reflect the relative positions of the joints in 3D.

    If the wrist is at (-9.164, 1.150, 13.432) and the index finger MCP is at (-0.019, -0.039, 0.180), the ratio of their x-coordinates (-9.164/-0.019 ≈ 482.3) tells us that the wrist is about 482 times farther from the origin than the index finger MCP, which is physically impossible for a human hand.



# To Test

docker exec -it d435i_docker_ros-realsense-1 bash
docker exec -it d435i_docker_ros-topic_reader-1 bash

source /opt/ros/humble/setup.bash

ros2 topic list

## Testing with RVIZ

* To view the camera feed, you can use RViz2:
```bash
source /opt/ros/humble/setup.bash
cd workspace/workspace
    ros2 run rviz2 rviz2

source /opt/ros/humble/setup.bash
cd workspace/workspace
ros2 run rviz2 rviz2 -d d435i.rviz

source /opt/ros/humble/setup.bash
cd workspace/workspace
ros2 run rviz2 rviz2 -d hand_marker_array.rviz
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