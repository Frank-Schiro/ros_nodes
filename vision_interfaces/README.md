# Overview

These are custum messages. Currently only for the hand object, which keeps track of the coordinates for a human hand.


```bash
# HandLandmark2D.msg
string name                    # e.g. "WRIST", "THUMB_TIP"
int32 pixel_x
int32 pixel_y

# Hand2D.msg
std_msgs/Header header        # timestamp + frame_id (camera_color_optical_frame)
string hand_id               
string handedness            
HandLandmark2D[] landmarks   
float64 tracking_confidence
string tracking_status      

# HandDetection2D.msg
std_msgs/Header header       # timestamp + frame_id (camera_color_optical_frame)
Hand2D[] hands              
uint32 image_width
uint32 image_height

# HandLandmark3D.msg
string name                          
geometry_msgs/Point position        
geometry_msgs/Quaternion orientation
float64 position_confidence        

# Hand3D.msg
std_msgs/Header header             # timestamp + frame_id (camera_link)
string hand_id                    
HandLandmark3D[] landmarks        
geometry_msgs/Pose palm_pose      

# HandDetection3D.msg
std_msgs/Header header            # timestamp + frame_id (camera_link)
Hand3D[] hands
```