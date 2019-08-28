cmake_minimum_required(VERSION 2.8.3)
project(roboteam_world)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

## Enable c++17 support.
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -g ${CMAKE_CXX_FLAGS} -O0")

#
#add_library(dummy_world STATIC src/dummy_world/dummy_world_main.cpp ${SOURCES})
#target_link_libraries(dummy_world ${ARMADILLO_LIBRARIES})

####################
## Filtered world ##
####################

#add_library(filtered_world STATIC src/filtered_world/filtered_world_main.cpp ${SOURCES})
#target_link_libraries(filtered_world ${ARMADILLO_LIBRARIES})


####################
## Kalman world   ##
####################
add_library(kalman_world STATIC
        src/kalman/kalman_world_main.cpp
        src/utils/predictor.cpp
        src/utils/ball.cpp
        src/utils/robot.cpp
        src/ros_handler.cpp
        src/kalman/kalmanFilter.cpp
        src/kalman/kalmanObject.cpp
        src/kalman/kalmanBall.cpp
        src/kalman/kalmanUs.cpp
        src/kalman/kalmanThem.cpp
        src/filtered_world/filtered_world.cpp
        src/dummy_world/world_dummy.cpp)


target_include_directories(kalman_world PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src )

target_link_libraries(kalman_world ${ARMADILLO_LIBRARIES} roboteam_utils roboteam_proto)

###########
## Tests ##
###########

#catkin_add_gtest(world_test
#    test/main.cpp
#        test/KalmanTest.cpp
#        test/BasicTest.cpp
#        test/WorldTest.cpp
#        src/filtered_world/filtered_world.cpp
#
#        )
#target_link_libraries(world_test
#    roboteam_world_srcs
#    ${catkin_LIBRARIES}
#        ${ARMADILLO_LIBRARIES}
#
#        )
#add_dependencies(world_test
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS}
#    )
