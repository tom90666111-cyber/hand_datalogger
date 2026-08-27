cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)
project(twincat_talker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  serial
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES trajectory_generator trajectory_uploader hand_streamer
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_filters serial
)

add_subdirectory(dep/ADS)

include_directories(
  include
  dep/ADS/AdsLib
  dep/ADS/AdsLibOOI
  ${catkin_INCLUDE_DIRS}
)

add_library(trajectory_generator
  src/lib/trajectory_generator.cpp
)

add_library(trajectory_uploader
  src/lib/trajectory_uploader.cpp
)
target_link_libraries(trajectory_uploader
  libadscxx
  ${catkin_LIBRARIES}
)

add_library(hand_streamer
  src/lib/hand_streamer.cpp
)
target_link_libraries(hand_streamer
  libadscxx
  ${catkin_LIBRARIES}
)

add_executable(twincat_talker src/main.cc)

target_link_libraries(twincat_talker
    libadscxx
    ${catkin_LIBRARIES}
)

add_executable(twincat_node src/twincat_node.cpp)
target_link_libraries(twincat_node
    libadscxx
    ${catkin_LIBRARIES}
)

add_executable(TCwrite src/TCwrite.cpp)
target_link_libraries(TCwrite
    libadscxx
    ${catkin_LIBRARIES}
)

add_executable(trajectory_file_writer src/trajectory_file_writer.cpp)
target_link_libraries(trajectory_file_writer
    libadscxx
    ${catkin_LIBRARIES}
)

# 假设 optitrack_node 的源文件是 src/optitrack_node.cpp
add_executable(optitrack_node src/optitrack_node.cpp)
target_link_libraries(optitrack_node
    ${catkin_LIBRARIES}
    # 如果 optitrack_node 也需要 ADS 库，则加上 libadscxx
)

add_executable(data_logger src/data_logger.cpp)
target_link_libraries(data_logger ${catkin_LIBRARIES})

add_executable(single_experiment_manager src/single_experiment_manager.cpp)
target_link_libraries(single_experiment_manager
  trajectory_generator
  hand_streamer
  libadscxx
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_trajectory_generator test/test_trajectory_generator.cpp)
  if(TARGET test_trajectory_generator)
    target_link_libraries(test_trajectory_generator trajectory_generator)
  endif()

  catkin_add_gtest(test_current_job_monitor test/test_current_job_monitor.cpp)
endif()

install(TARGETS
   twincat_talker
   twincat_node
   optitrack_node
   data_logger
   trajectory_file_writer
   single_experiment_manager
   trajectory_generator
   trajectory_uploader
   hand_streamer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
