cmake_minimum_required(VERSION 3.0.2)
project(rm_deploy_stream)
set(CMAKE_CXX_STANDARD 17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  serial
  message_generation
)
set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_message_files(
  FILES
  VideoPacket.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rm_deploy_stream_core rm_deploy_stream_nodelet
  CATKIN_DEPENDS roscpp nodelet image_transport cv_bridge sensor_msgs std_msgs serial message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

add_library(rm_deploy_stream_core
  src/video_encoder_node.cpp
  src/serial_sender_node.cpp
)
target_link_libraries(rm_deploy_stream_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
)
add_dependencies(rm_deploy_stream_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(rm_deploy_stream_nodelet src/video_encoder_nodelet.cpp)
target_link_libraries(rm_deploy_stream_nodelet rm_deploy_stream_core ${catkin_LIBRARIES})
add_dependencies(rm_deploy_stream_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rm_deploy_stream_main src/main.cpp)
target_link_libraries(rm_deploy_stream_main rm_deploy_stream_core ${catkin_LIBRARIES})
add_dependencies(rm_deploy_stream_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
