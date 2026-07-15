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
    CATKIN_DEPENDS roscpp sensor_msgs serial
)

add_subdirectory(dep/ADS)

include_directories(dep/ADS/AdsLib dep/ADS/AdsLibOOI ${catkin_INCLUDE_DIRS})

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

add_executable(sensor_read_five_tips src/sensor_read_five_tips.cpp)
target_link_libraries(sensor_read_five_tips
  ${catkin_LIBRARIES}
)

install(TARGETS twincat_talker
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)