cmake_minimum_required(VERSION 2.8.3)
project(roboteam_world)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  roboteam_msgs
  roboteam_utils
)

## Enable c++11 support.
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g ${CMAKE_CXX_FLAGS} -O0")

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
    INCLUDE_DIRS
        include
    LIBRARIES
        roboteam_world_srcs
    CATKIN_DEPENDS
        message_runtime
    DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  "include"
)

####################
## Source library ##
####################

add_library(roboteam_world_srcs 
    # src/
        src/utils/predictor.cpp
        src/utils/ball.cpp
        src/utils/robot.cpp
        src/ros_handler.cpp
        src/kalman/kalmanFilter.cpp
        src/kalman/kalmanObject.cpp

    # src/world
        src/filtered_world/filtered_world.cpp
        src/dummy_world/world_dummy.cpp

    )

add_dependencies(roboteam_world_srcs
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
target_link_libraries(roboteam_world_srcs 
    ${catkin_LIBRARIES}
    )

#################
## Dummy world ##
#################

add_executable(dummy_world
        src/dummy_world/dummy_world_main.cpp
    )
target_link_libraries(dummy_world 
    roboteam_world_srcs
    ${catkin_LIBRARIES}
    )
add_dependencies(dummy_world 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    )

####################
## Filtered world ##
####################

add_executable(filtered_world
        src/filtered_world/filtered_world_main.cpp
    )
target_link_libraries(filtered_world
    roboteam_world_srcs
    ${catkin_LIBRARIES}
    )
add_dependencies(filtered_world 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    )

###########
## Tests ##
###########

catkin_add_gtest(world_test
    test/main.cpp
        test/KalmanTest.cpp
        test/BasicTest.cpp
        test/WorldTest.cpp
        src/filtered_world/filtered_world.cpp

        )
target_link_libraries(world_test
    roboteam_world_srcs
    ${catkin_LIBRARIES}
    )
add_dependencies(world_test 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
