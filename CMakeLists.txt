cmake_minimum_required(VERSION 3.10)
project(roboteam_world)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

add_library(net STATIC
        src/net/netraw.cpp
        src/net/robocup_ssl_client.cpp)

add_executable(kalman_world
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

target_link_libraries(kalman_world
        PRIVATE ${ARMADILLO_LIBRARIES}
        PRIVATE net
        PRIVATE roboteam_utils
        PUBLIC roboteam_proto)

