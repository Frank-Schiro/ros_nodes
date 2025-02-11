# Hand3D.msg
std_msgs/Header header             # timestamp + frame_id (camera_link)
string hand_id                    
HandLandmark3D[] landmarks        
geometry_msgs/Pose palm_pose      