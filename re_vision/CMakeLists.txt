cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# add zlib
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(ZLIB REQUIRED zlib)
#include_directories(${ZLIB_INCLUDE_DIRS})
#link_directories(${ZLIB_LIBRARY_DIRS})


# build 3rd party libraries

#include_directories(src/3rdparty/DUtils)
#include_directories(src/3rdparty/DUtilsCV)
#include_directories(src/3rdparty/DVision)
include_directories(include/re_vision)
include_directories(src/3rdparty/RapidXml)
include_directories(src/3rdparty/epnp)

# DUtils
rosbuild_add_library(DUtils
  src/3rdparty/DUtils/BinaryFile.cpp     src/3rdparty/DUtils/LineFile.cpp  
  src/3rdparty/DUtils/Random.cpp
  src/3rdparty/DUtils/FileFunctions.cpp  src/3rdparty/DUtils/Profiler.cpp  
  src/3rdparty/DUtils/Timestamp.cpp      src/3rdparty/DUtils/StringFunctions.cpp
)

# DUtilsCV
rosbuild_add_library(DUtilsCV
  src/3rdparty/DUtilsCV/Drawing.cpp   src/3rdparty/DUtilsCV/GUI.cpp  
  src/3rdparty/DUtilsCV/Transformations.cpp
  src/3rdparty/DUtilsCV/Geometry.cpp  src/3rdparty/DUtilsCV/IO.cpp
)

# DVision
rosbuild_add_library(DVision
  src/3rdparty/DVision/PatchFile.cpp       
  src/3rdparty/DVision/PMVSCamera.cpp
  src/3rdparty/DVision/BundleCamera.cpp  src/3rdparty/DVision/PixelPointFile.cpp  
  src/3rdparty/DVision/SurfSet.cpp
  src/3rdparty/DVision/Matches.cpp       src/3rdparty/DVision/PLYFile.cpp
)

# build a library to share code between ObjectDetector and ObjectDetectorDummy
rosbuild_add_library(ObjectDetectorFunctions
  src/ObjectDetectorClass.cpp
  src/ObjectDetectorProvider.cpp
	src/CameraBridge.cpp src/CameraBridgeFactory.cpp
	src/MetaFile.cpp src/ObjectModel.cpp 
	src/PlanarVisualizationModel.cpp src/PointCloudVisualizationModel.cpp
	src/ObjectDetectorMethod.cpp src/SurfPlanarDetector.cpp src/Surf3DDetector.cpp
	src/VisualizationManager.cpp
	src/3rdparty/epnp/epnp.cpp
	src/debug.cpp
)

# build ObjectDetector
rosbuild_add_executable(ObjectDetector 
  src/ObjectDetector.cpp 
)

#target_link_libraries(ObjectDetector Ferns ${ZLIB_LIBRARIES})
#link_directories(/usr/local/lib)
target_link_libraries(ObjectDetector ObjectDetectorFunctions 
  DVision DUtilsCV DUtils)

# build ObjectDetectorDummy
rosbuild_add_executable(ObjectDetectorDummy
  src/ObjectDetectorDummy.cpp
)
target_link_libraries(ObjectDetectorDummy ObjectDetectorFunctions 
  DVision DUtilsCV DUtils)

# --- Optional nodes ---------------------------------------------------------

# PlanarModelCreator
rosbuild_add_executable(createPlanarModel
  src/PlanarModelCreator/createPlanarModel.cpp
)

target_link_libraries(createPlanarModel ObjectDetectorFunctions
  DVision DUtilsCV DUtils)


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# CubeModelCreator
rosbuild_add_executable(createCubeModel
  src/CubeModelCreator/createCubeModel.cpp
)

target_link_libraries(createCubeModel ObjectDetectorFunctions
  DVision DUtilsCV DUtils)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# PointCloudModelCreator (several apps)

rosbuild_add_executable(changeReferenceSystem
  src/PointCloudModelCreator/changeReferenceSystem.cpp
)
target_link_libraries(changeReferenceSystem DVision DUtilsCV DUtils)

rosbuild_add_executable(createPointCloudModel
  src/PointCloudModelCreator/createPointCloudModel.cpp
)
target_link_libraries(createPointCloudModel DVision DUtilsCV DUtils)

rosbuild_add_executable(computePixelPoints2
  src/PointCloudModelCreator/computePixelPoints2.cpp
  src/PointCloudModelCreator/Mask.cpp
)
target_link_libraries(computePixelPoints2 DVision DUtilsCV DUtils)

rosbuild_add_executable(computePixelPoints
  src/PointCloudModelCreator/computePixelPoints.cpp
  src/PointCloudModelCreator/Mask.cpp
)
target_link_libraries(computePixelPoints DVision DUtilsCV DUtils)

rosbuild_add_executable(createPlyPoints
  src/PointCloudModelCreator/createPlyPoints.cpp
)
target_link_libraries(createPlyPoints DVision DUtilsCV DUtils)

rosbuild_add_executable(extractOutlineFromImages
  src/PointCloudModelCreator/extractOutlineFromImages.cpp
  src/PointCloudModelCreator/Mask.cpp
)
target_link_libraries(extractOutlineFromImages DVision DUtilsCV DUtils)

rosbuild_add_executable(extractSurf
  src/PointCloudModelCreator/extractSurf.cpp
  src/PointCloudModelCreator/Mask.cpp
)
target_link_libraries(extractSurf DVision DUtilsCV DUtils)

rosbuild_add_executable(generateMeta
  src/PointCloudModelCreator/generateMeta.cpp
)
target_link_libraries(generateMeta ObjectDetectorFunctions DVision DUtilsCV DUtils)

rosbuild_add_executable(removeBackgroundPoints
  src/PointCloudModelCreator/removeBackgroundPoints.cpp
  src/PointCloudModelCreator/Mask.cpp
)
target_link_libraries(removeBackgroundPoints DVision DUtilsCV DUtils)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# ModelTools

rosbuild_add_executable(loadModelIntoVSlam src/ModelTools/loadModelIntoVSlam.cpp)
target_link_libraries(loadModelIntoVSlam ObjectDetectorFunctions
  DVision DUtilsCV DUtils)
  
rosbuild_add_executable(getModelInfo src/ModelTools/getModelInfo.cpp)
target_link_libraries(getModelInfo ObjectDetectorFunctions
  DVision DUtilsCV DUtils)
  
rosbuild_add_executable(showModel src/ModelTools/showModel.cpp)
target_link_libraries(showModel ObjectDetectorFunctions
  DVision DUtilsCV DUtils)

rosbuild_add_executable(changeScaleFactor src/ModelTools/changeScaleFactor.cpp)
target_link_libraries(changeScaleFactor ObjectDetectorFunctions
  DVision DUtilsCV DUtils)

rosbuild_add_executable(testModel src/ModelTools/testModel.cpp)
target_link_libraries(testModel ObjectDetectorFunctions
  DVision DUtilsCV DUtils)

rosbuild_add_executable(removeModelFace src/ModelTools/removeModelFace.cpp)
target_link_libraries(removeModelFace ObjectDetectorFunctions
  DVision DUtilsCV DUtils)

add_custom_command(TARGET createPointCloudModel
	POST_BUILD
	COMMAND cp `rospack find re_vision`/src/PointCloudModelCreator/createModel.sh `rospack find re_vision`/bin
)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# TestObjectDetector
rosbuild_add_executable(TestObjectDetector 
  src/TestObjectDetector.cpp
)

target_link_libraries(TestObjectDetector ObjectDetectorFunctions
  DVision DUtilsCV DUtils)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


