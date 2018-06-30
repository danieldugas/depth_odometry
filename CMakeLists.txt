cmake_minimum_required(VERSION 2.8.3)
project(depth_odometry)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf rospy std_msgs sensor_msgs geometry_msgs
  glog_catkin )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES depth_odometry
#  CATKIN_DEPENDS dbw_mkz_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_compile_options(-std=c++11)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(depth_odometry
#   src/${PROJECT_NAME}/depth_odometry.cpp
# )

## Declare a C++ executable
add_executable(depth_odometry src/depth_odometry.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(depth_odometry ${catkin_LIBRARIES})
