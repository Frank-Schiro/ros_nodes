We are creating new hand landmarks here
This requires also this package: https://github.com/ros-perception/vision_msgs
# HandLandmark.msg 
std_msgs/Header header
string name                          # e.g. "WRIST", "THUMB_TIP"
geometry_msgs/Point2D pixel_coords
geometry_msgs/Point position         # 3D position in camera frame (if available)
geometry_msgs/Quaternion orientation # 3D orientation (if available)
float64 position_confidence         # Confidence in the 3D position
float64 detection_confidence        # Confidence in the 2D detection

# Hand.msg
string hand_id                      # Unique identifier for this hand
string handedness                   # "LEFT" or "RIGHT"
HandLandmark[] landmarks           # Array of landmarks for this hand
geometry_msgs/Pose palm_pose       # Overall palm pose in camera frame
float64 tracking_confidence        # Overall confidence for this hand
string tracking_status             # "TRACKING", "LOST", "INITIALIZING"

# HandDetection.msg
std_msgs/Header header            # Timestamp and frame_id
Hand[] hands                      # Array of detected hands
string camera_frame_id           # Frame ID of the camera that captured this
bool has_depth_data             # Whether 3D position data is available
uint32 image_width
uint32 image_height



