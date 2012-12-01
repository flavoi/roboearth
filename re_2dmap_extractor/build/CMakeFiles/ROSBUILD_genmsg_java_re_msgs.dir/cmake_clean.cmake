FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/re_2dmap_extractor/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_java_re_msgs"
  "../msg_gen/java/ros/pkg/re_msgs/msg/Pose2DStamped.java"
  "../msg_gen/java/ros/pkg/re_msgs/msg/Pixel.java"
  "../msg_gen/java/ros/pkg/re_msgs/msg/SeenObject.java"
  "../msg_gen/java/ros/pkg/re_msgs/msg/File.java"
  "../msg_gen/java/ros/pkg/re_msgs/msg/StringArray.java"
  "../msg_gen/java/ros/pkg/re_msgs/msg/SeenObjectArray.java"
  "../msg_gen/java/ros/pkg/re_msgs/msg/RosFile.java"
  "../msg_gen/java/ros/pkg/re_msgs/msg/DetectedObject.java"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_java_re_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
