cmake_minimum_required(VERSION 3.0.2)
project(miradar_node)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  image_transport
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
  nav_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rostest REQUIRED)

add_message_files(
  FILES
  PPI.msg
  PPIData.msg
)
generate_messages(
  DEPENDENCIES
  sensor_msgs std_msgs visualization_msgs geometry_msgs
)

generate_dynamic_reconfigure_options(
    cfg/MiRadar.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



add_executable(${PROJECT_NAME} src/miradar_node.cpp)
add_executable(scan_matching src/scan_matching.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

target_link_libraries(scan_matching
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)


include_directories(tests/include)
catkin_add_gtest(miradartest tests/miradartest.cpp)
if(CATKIN_ENABLE_TESTING)
  add_rostest(tests/dynconf.test)
endif()
execute_process(COMMAND cppcheck src/*.cpp include/*.h --language=c++ --std=c++11 --xml-version=2 --output-file=${CMAKE_CURRENT_SOURCE_DIR}/miradar_node/lint.xml
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/miradar_node
)
