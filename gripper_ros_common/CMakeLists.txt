cmake_minimum_required(VERSION 3.0.2)
project(gripper_ros_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  std_srvs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PositionSetpoint.msg
  VelocitySetpoint.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
 CATKIN_DEPENDS message_runtime std_msgs std_srvs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

