cmake_minimum_required(VERSION 2.8.3)
project(rtsp_ffmpeg)
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-D__STDC_CONSTANT_MACROS")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
  nodelet
  dynamic_reconfigure
)

find_package(OpenCV)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)

find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARY swscale)

generate_dynamic_reconfigure_options(
  cfg/RTSPFFmpeg.cfg)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIR} 
  ${AVFORMAT_INCLUDE_DIR} 
  ${AVUTIL_INCLUDE_DIR} 
  ${AVDEVICE_INCLUDE_DIR}
  ${SWSCALE_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/ffmpegdecoder.cpp
  src/rtsp_ffmpeg.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${AVDEVICE_LIBRARY} ${SWSCALE_LIBRARY})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(rtsp_ffmpeg_node src/rtsp_ffmpeg_node.cpp)
target_link_libraries(rtsp_ffmpeg_node ${catkin_LIBRARIES})
set_target_properties(rtsp_ffmpeg_node PROPERTIES OUTPUT_NAME rtsp_ffmpeg)

# install(TARGETS video_stream_node ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(PROGRAMS
#   scripts/test_video_resource.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(DIRECTORY launch test
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   USE_SOURCE_PERMISSIONS)

# install(FILES nodelet_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# if (CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest(test/test_video_file.test)
#   add_rostest(test/test_mjpg_stream.test)
#   add_rostest(test/test_rtsp_stream.test)
# endif()
