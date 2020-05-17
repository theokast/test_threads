cmake_minimum_required(VERSION 2.8.3)

project(test_threads)

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 -DLINUX -Wno-narrowing")

# Build in release mode
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  autharl_core
  autharl_ati_sensor
  lwr_robot
  pcl_ros
  pcl_conversions
  bhand_robot
  bhand_controllers
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES smartsurg_dace_controller
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
 /home/user/dora/sigma_ws/src/sdk-3.9.1/include
)
link_directories( /home/user/dora/sigma_ws/src/sdk-3.9.1/lib/release/lin-x86_64-gcc)

add_executable(${PROJECT_NAME}_node
 src/main.cpp
src/threadSmall.cpp
src/threadLarge.cpp

)


set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  dhd
  drd
  usb-1.0 -lrt
  pthread
)
