FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/re_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/re_msgs/Pose2DStamped.h"
  "../msg_gen/cpp/include/re_msgs/Pixel.h"
  "../msg_gen/cpp/include/re_msgs/SeenObject.h"
  "../msg_gen/cpp/include/re_msgs/File.h"
  "../msg_gen/cpp/include/re_msgs/StringArray.h"
  "../msg_gen/cpp/include/re_msgs/SeenObjectArray.h"
  "../msg_gen/cpp/include/re_msgs/RosFile.h"
  "../msg_gen/cpp/include/re_msgs/DetectedObject.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
