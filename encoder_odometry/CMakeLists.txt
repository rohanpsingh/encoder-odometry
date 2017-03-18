cmake_minimum_required(VERSION 2.8.3)
project(encoder_odometry)

find_package(catkin REQUIRED COMPONENTS
	tf
	geometry_msgs
	nav_msgs
        roscpp
        rospy
        std_msgs
)


catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(encoder_odometry
#   src/${PROJECT_NAME}/encoder_odometry.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(encoder_odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry
   ${catkin_LIBRARIES}
)

