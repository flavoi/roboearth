FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gensrv_java_vision_srvs"
  "../srv_gen/java/ros/pkg/vision_srvs/srv/srvjlo.java"
  "../srv_gen/java/ros/pkg/vision_srvs/srv/cop_call.java"
  "../srv_gen/java/ros/pkg/vision_srvs/srv/register_jlo_callback.java"
  "../srv_gen/java/ros/pkg/vision_srvs/srv/cop_get_object_shape.java"
  "../srv_gen/java/ros/pkg/vision_srvs/srv/clip_polygon.java"
  "../srv_gen/java/ros/pkg/vision_srvs/srv/cop_save.java"
  "../srv_gen/java/ros/pkg/vision_srvs/srv/cop_add_collision.java"
  "../srv_gen/java/ros/pkg/vision_srvs/srv/cop_get_methods_list.java"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_java_vision_srvs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
