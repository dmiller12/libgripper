cmake_minimum_required(VERSION 3.10)
project(gripper_ros)

# Find standard ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

# This is the magic line that finds your non-ROS library
find_package(gripper REQUIRED)

catkin_package(
	CATKIN_DEPENDS roscpp
)

# Define the ROS node executable
add_executable(bhand_node src/bhand_node.cpp)

include_directories(${catkin_INCLUDE_DIRS})

# Link the node against Catkin libs AND your gripper library's modern target
target_link_libraries(bhand_node
  ${catkin_LIBRARIES}
  gripper::gripper
)
