cmake_minimum_required(VERSION 3.0.2)
project(quad_ctrl_mavros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  tf2
  roscpp
  message_generation
)

##########################
## Declare ROS services ##
##########################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AssertReadiness.srv
  RequestDeparture.srv
)

## Generate added messages and services
generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime geometry_msgs mavros_msgs tf2 roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare C++ executables
add_executable(${PROJECT_NAME}_node src/main.cpp src/uav.cpp src/utils.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
