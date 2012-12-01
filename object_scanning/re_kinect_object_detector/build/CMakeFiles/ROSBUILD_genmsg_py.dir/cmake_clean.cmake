FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/re_kinect_object_detector/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/re_kinect_object_detector/msg/__init__.py"
  "../src/re_kinect_object_detector/msg/_DetectionResult.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
