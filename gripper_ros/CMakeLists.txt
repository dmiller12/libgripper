cmake_minimum_required(VERSION 3.10)
project(gripper_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  gripper_ros_common
)

find_package(gripper REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime std_srvs
)

add_executable(bhand_node src/bhand_node.cpp)
add_dependencies(bhand_node ${catkin_EXPORTED_TARGETS})

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(bhand_node
  ${catkin_LIBRARIES}
  gripper::gripper
)
