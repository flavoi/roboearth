cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()


rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
rosbuild_find_ros_package(mod_srdl)
set( SRDL_PATH_PREFIX ${mod_srdl_PACKAGE_PATH} )
rosbuild_find_ros_package(re_ontology)
set( RE_PATH_PREFIX ${re_ontology_PACKAGE_PATH} )


#######################################
# reset local paths in Prolog files

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

#######################################
# Java stuff

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

# external dependencies in other packages
rosbuild_find_ros_package(rosjava_jni)
add_classpath(${rosjava_jni_PACKAGE_PATH}/bin)

rospack_add_java_executable(ReVisionROSClient org.roboearth.re_ontology.ReVisionROSClient)
# rospack_add_java_executable(ReVisionDummyPublisher org.roboearth.re_ontology.ReVisionDummyPublisher)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
