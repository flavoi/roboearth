FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/re_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/re_msgs/msg/__init__.py"
  "../src/re_msgs/msg/_Pose2DStamped.py"
  "../src/re_msgs/msg/_Pixel.py"
  "../src/re_msgs/msg/_SeenObject.py"
  "../src/re_msgs/msg/_File.py"
  "../src/re_msgs/msg/_StringArray.py"
  "../src/re_msgs/msg/_SeenObjectArray.py"
  "../src/re_msgs/msg/_RosFile.py"
  "../src/re_msgs/msg/_DetectedObject.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
