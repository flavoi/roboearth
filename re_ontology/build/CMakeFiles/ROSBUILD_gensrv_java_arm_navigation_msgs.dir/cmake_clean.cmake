FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gensrv_java_arm_navigation_msgs"
  "../srv_gen/java/ros/pkg/arm_navigation_msgs/srv/GetJointTrajectoryValidity.java"
  "../srv_gen/java/ros/pkg/arm_navigation_msgs/srv/GetMotionPlan.java"
  "../srv_gen/java/ros/pkg/arm_navigation_msgs/srv/GetPlanningScene.java"
  "../srv_gen/java/ros/pkg/arm_navigation_msgs/srv/FilterJointTrajectoryWithConstraints.java"
  "../srv_gen/java/ros/pkg/arm_navigation_msgs/srv/FilterJointTrajectory.java"
  "../srv_gen/java/ros/pkg/arm_navigation_msgs/srv/GetCollisionObjects.java"
  "../srv_gen/java/ros/pkg/arm_navigation_msgs/srv/SetPlanningSceneDiff.java"
  "../srv_gen/java/ros/pkg/arm_navigation_msgs/srv/GetRobotState.java"
  "../srv_gen/java/ros/pkg/arm_navigation_msgs/srv/GetRobotTrajectoryValidity.java"
  "../srv_gen/java/ros/pkg/arm_navigation_msgs/srv/GetStateValidity.java"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_java_arm_navigation_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
