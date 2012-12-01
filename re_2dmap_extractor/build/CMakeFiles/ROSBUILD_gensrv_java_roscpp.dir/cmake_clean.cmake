FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/re_2dmap_extractor/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_java_roscpp"
  "../srv_gen/java/ros/pkg/roscpp/srv/GetLoggers.java"
  "../srv_gen/java/ros/pkg/roscpp/srv/Empty.java"
  "../srv_gen/java/ros/pkg/roscpp/srv/SetLoggerLevel.java"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_java_roscpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
