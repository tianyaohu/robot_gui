cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotinfo_msgs
  roscpp
  std_srvs
  OpenCV REQUIRED
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_gui
#  CATKIN_DEPENDS geometry_msgs nav_msgs robotinfo_msgs roscpp std_srvs
#  DEPENDS system_lib
)

###########

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_library(robot_gui_lib src/robot_gui.cpp)
# add_executable(window_node src/window_example.cpp)
# target_link_libraries(window_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

add_executable(ui_node src/robot_gui.cpp)

target_link_libraries(ui_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

