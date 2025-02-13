# Hand2D.msg
std_msgs/Header header        # timestamp + frame_id (camera_color_optical_frame)
string hand_id               
string handedness            
HandLandmark2D[] landmarks   
float64 tracking_confidence
