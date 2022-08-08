cmake_minimum_required(VERSION 3.0.2)
project(state_estimation)

## Compile as C++14
add_compile_options(-std=c++14)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

################
# Dependencies #
################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  px4_control_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES state_estimation
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs px4_control_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Runnables
# State Observer
add_executable(state_observer_node
  src/state_observer_node.cc
  src/state_observer.cc
)

add_dependencies(state_observer_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(state_observer_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
