FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/re_kinect_object_detector/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/DetectionResult.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_DetectionResult.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
