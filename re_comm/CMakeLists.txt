cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_gensrv()

add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/commons-codec-1.4.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/commons-logging-1.1.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpclient-4.1.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpclient-cache-4.1.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpcore-4.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpmime-4.1.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/owlapi/owlapi-bin.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/hermit/HermiT.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/jgrapht/jgrapht-jdk1.6.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/mysql/mysql-connector-java-5.1.13-bin.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/argo/argo-2.8.jar)
add_classpath(${PROJECT_SOURCE_DIR}/bin)

add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/commons-codec-1.4.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/commons-logging-1.1.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpclient-4.1.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpclient-cache-4.1.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpcore-4.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/httpcomponents-client/httpmime-4.1.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/owlapi/owlapi-bin.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/hermit/HermiT.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/jgrapht/jgrapht-jdk1.6.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/mysql/mysql-connector-java-5.1.13-bin.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/argo/argo-2.8.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/bin)

rosbuild_find_ros_package(knowrob_common)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/guava-r09.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/guava-r09.jar)
add_classpath(${knowrob_common_PACKAGE_PATH}/bin)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/bin)

rosbuild_find_ros_package(mod_vis)
add_classpath(${mod_vis_PACKAGE_PATH}/bin)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/core.jar)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/controlP5.jar)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/vecmath.jar)

add_runtime_classpath(${mod_vis_PACKAGE_PATH}/bin)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/core.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/controlP5.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/vecmath.jar)


rosbuild_find_ros_package(comp_cop)
add_classpath(${comp_cop_PACKAGE_PATH}/bin)
add_runtime_classpath(${comp_cop_PACKAGE_PATH}/bin)


add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
rospack_add_java_executable(run roboearth.wp5.Main)
rospack_add_java_executable(test roboearth.wp5.util.Util)
rospack_add_java_executable(CopRoboEarthInterface roboearth.wp5.CopRoboEarthInterface)
rospack_add_java_executable(UnizarRoboEarthInterface roboearth.wp1.UnizarRoboEarthInterface)
rospack_add_java_executable(set_updated_expedit roboearth.wp5.REDummyUploader)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# adapt paths in Prolog files
# 
rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
rosbuild_find_ros_package(re_ontology)
set( RE_PATH_PREFIX ${re_ontology_PACKAGE_PATH} )

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )
