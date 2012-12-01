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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(OpenCV REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtXML REQUIRED)

include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

rosbuild_add_boost_directories()

set(UI_SRCS
    src/mainwindow.ui
)

set(MOC_SRCS
    src/mainwindow.h
    src/comthread.h
)

set(SRCS
    src/mainwindow.cpp
    src/comthread.cpp
    src/owldescriptioncreator.cpp
    src/re_visionmodelcreator.cpp
)

qt4_wrap_ui(SRCS ${UI_SRCS})
qt4_wrap_cpp(SRCS ${MOC_SRCS})


include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${QT_INCLUDE_DIR}
    ${QT_QTGUI_INCLUDE_DIR}
    ${QT_QTCORE_INCLUDE_DIR}
    ${QT_INCLUDE_DIRS}
)

include_directories(/usr/include/vtk-5.8)

rosbuild_add_library(qminizip
    src/qminizip.cpp
    src/minizip/zip.c
    src/minizip/unzip.c
    src/minizip/ioapi.c
)
target_link_libraries(qminizip
    ${QT_QTCORE_LIBRARIES}
    z
)

rosbuild_add_executable(record_gui ${SRCS} src/main.cpp)

target_link_libraries(record_gui
    ${OpenCV_LIBS}
    ${QT_LIBRARIES}
    ${QT_QTGUI_LIBRARIES}
    ${QT_QTCORE_LIBRARIES}
    qminizip
    libvtkCommon.so libvtkFiltering.so libvtkRendering.so
)
rosbuild_add_executable(upload_object_model
    src/upload_object_model.cpp
    src/owldescriptioncreator.cpp
)
rosbuild_link_boost(upload_object_model program_options)
target_link_libraries(upload_object_model
    ${QT_QTCORE_LIBRARIES}
    ${QT_QTXML_LIBRARIES}
    qminizip
    libvtkCommon.so libvtkFiltering.so libvtkRendering.so
)
