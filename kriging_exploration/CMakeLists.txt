cmake_minimum_required(VERSION 2.8.3)
project(kriging_exploration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_python_setup()

#add_message_files(
#   FILES
#)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetTsp.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES kriging_exploration
#  CATKIN_DEPENDS message_generation rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(tsp_service src/tsp.cpp)
target_link_libraries(tsp_service ${catkin_LIBRARIES})
add_dependencies(tsp_service kriging_exploration_generate_messages_cpp)


