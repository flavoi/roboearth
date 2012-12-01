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

find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
rosbuild_add_boost_directories()

include(${QT_USE_FILE})

set(UI_SRCS
    src/mainwindow.ui
    src/downloadobjectdialog.ui
)

set(MOC_SRCS
    src/comthread.h
    src/mainwindow.h
    src/downloadobjectdialog.h
)

set(SRCS
    src/main.cpp
    src/comthread.cpp
    src/mainwindow.cpp
    src/downloadobjectdialog.cpp
)

qt4_wrap_ui(SRCS ${UI_SRCS})
qt4_wrap_cpp(SRCS ${MOC_SRCS})


rosbuild_add_executable(detect ${SRCS} src/main.cpp)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${QT_INCLUDE_DIR}
    ${QT_QTGUI_INCLUDE_DIR}
    ${QT_QTCORE_INCLUDE_DIR}
    ${QT_INCLUDE_DIRS}
)

target_link_libraries(detect
    ${QT_LIBRARIES}
    ${QT_QTGUI_LIBRARIES}
    ${QT_QTCORE_LIBRARIES}
    qminizip
    tinyxml
)
