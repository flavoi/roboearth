# Information of object detected in an image by re_vision
#

# detected points in the image
Pixel[] points2d
# detected 3d points in the camera reference
geometry_msgs/Point[] points3d
# pose of the object in the camera reference
geometry_msgs/Pose pose
# detected 3d points in the model reference
geometry_msgs/Point[] points3d_model
# detected 3d points octave
int32[] octave
