cmake_minimum_required(VERSION 3.10)
project(roboteam_world)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(Armadillo REQUIRED)
SET(CMAKE_CXX_STANDARD 17)

add_library(net STATIC
        src/net/netraw.cpp
        src/net/robocup_ssl_client.cpp)

target_link_libraries(net
        PUBLIC roboteam_proto)

add_executable(roboteam_world
        src/main.cpp
        src/WorldHandler.cpp
        src/kalman/KalmanFilter.cpp
        src/kalman/KalmanObject.cpp
        src/kalman/KalmanBall.cpp
        src/kalman/KalmanRobot.cpp
        src/world/world.cpp
        src/world/world_data.cpp
        src/world/robot.cpp
        src/world/settings.cpp
        src/world/ball.cpp
        src/world/views/world_data_view.cpp include/roboteam_world/world/views/world_data_view.hpp)


target_include_directories(roboteam_world
        INTERFACE include
        PRIVATE ../roboteam_world/include
        PRIVATE ../roboteam_proto/include
        PRIVATE ../roboteam_ai/include
        PRIVATE src
        )

#including armadillo files
target_include_directories(roboteam_world
        PRIVATE ${ARMADILLO_INCLUDE_DIRS}
        )

target_link_libraries(roboteam_world
        PRIVATE ${ARMADILLO_LIBRARIES}
        PRIVATE net
        PRIVATE roboteam_utils
        PUBLIC roboteam_proto)

add_executable(world_tests
        src/WorldHandler.cpp
        src/kalman/KalmanFilter.cpp
        src/kalman/KalmanObject.cpp
        src/kalman/KalmanBall.cpp
        src/kalman/KalmanRobot.cpp
        test/KalmanTest.cpp
        test/main.cpp)

target_include_directories(world_tests
        INTERFACE include
        PRIVATE ../cmake-build-debug
        PRIVATE src
        )

#including armadillo files
target_include_directories(world_tests
        PRIVATE ${ARMADILLO_INCLUDE_DIRS}
        )


if(UNIX AND NOT APPLE)
    target_link_libraries(world_tests
            PRIVATE ${ARMADILLO_LIBRARIES}
            PRIVATE net
            PRIVATE roboteam_utils
            PRIVATE gtest
            PUBLIC roboteam_proto)
endif()

if(APPLE)
    target_link_libraries(world_tests
            PRIVATE ${ARMADILLO_LIBRARIES}
            PRIVATE net
            PRIVATE roboteam_utils
            PUBLIC roboteam_proto
            /usr/local/lib/libgtest.a
            /usr/local/lib/libgtest_main.a)
endif()
