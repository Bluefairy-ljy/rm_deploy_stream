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
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Protobuf REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)
pkg_check_modules(FFMPEG REQUIRED libavcodec libavutil libavformat libswscale)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS proto/referee.proto)

add_message_files(
  FILES
  VideoPacket.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${FFMPEG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROTOBUF_INCLUDE_DIR}
)

add_library(rm_deploy_stream_core
  src/video_encoder_node.cpp
  src/video_decoder_node.cpp
  ${PROTO_SRCS}
)
add_dependencies(rm_deploy_stream_core ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(rm_deploy_stream_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
  ${FFMPEG_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

add_executable(video_encoder src/video_encoder_main.cpp)
target_link_libraries(video_encoder rm_deploy_stream_core)

add_executable(video_decoder src/video_decoder_main.cpp)
target_link_libraries(video_decoder rm_deploy_stream_core)
