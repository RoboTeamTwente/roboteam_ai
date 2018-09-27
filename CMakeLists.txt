cmake_minimum_required(VERSION 3.1)
project(roboteam_ai)

find_package(catkin REQUIRED COMPONENTS
        roboteam_msgs
        roboteam_utils
        roboteam_world
        roscpp
        std_msgs
        actionlib_msgs
        actionlib
        uuid_msgs
        unique_id
        )


set(DANGER_FINDER_SOURCES
        roboteam_ai/src/danger_finder/CanShootModule.cpp
        roboteam_ai/src/danger_finder/CanShootModule.h
        roboteam_ai/src/danger_finder/DangerData.cpp
        roboteam_ai/src/danger_finder/DangerData.h
        roboteam_ai/src/danger_finder/DangerFinder.cpp
        roboteam_ai/src/danger_finder/DangerFinder.h
        roboteam_ai/src/danger_finder/DangerFinderConfig.cpp
        roboteam_ai/src/danger_finder/DangerFinderConfig.h
        roboteam_ai/src/danger_finder/DangerModule.cpp
        roboteam_ai/src/danger_finder/DangerModule.h
        roboteam_ai/src/danger_finder/DistanceModule.cpp
        roboteam_ai/src/danger_finder/DistanceModule.h
        roboteam_ai/src/danger_finder/FreeModule.cpp
        roboteam_ai/src/danger_finder/FreeModule.h
        roboteam_ai/src/danger_finder/HasBallModule.cpp
        roboteam_ai/src/danger_finder/HasBallModule.h
        roboteam_ai/src/danger_finder/OrientationModule.cpp
        roboteam_ai/src/danger_finder/OrientationModule.h
        roboteam_ai/src/danger_finder/WillReceiveBallModule.cpp
        roboteam_ai/src/danger_finder/WillReceiveBallModule.h
        )


set(IO_SOURCES
        roboteam_ai/src/io/IOManager.h
        roboteam_ai/src/io/IOManager.cpp
        roboteam_ai/src/io/StrategyIOManager.cpp
        roboteam_ai/src/io/StrategyIOManager.h
        roboteam_ai/src/io/RoleIOManager.cpp
        roboteam_ai/src/io/RoleIOManager.h)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory(roboteam_ai/src)
add_dependencies(bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# StrategyNode
add_executable(StrategyNode
        roboteam_ai/src/StrategyNode.cpp
        ${IO_SOURCES}
        ${DANGER_FINDER_SOURCES})
add_dependencies(StrategyNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(StrategyNode ${catkin_LIBRARIES} bt)


#############
## Testing ##
#############
catkin_add_gtest(bt_test
        roboteam_ai/test/main.cpp
        roboteam_ai/test/BtTest.cpp
        )

target_link_libraries(bt_test
        ${catkin_LIBRARIES}
        ${DANGER_FINDER_SOURCES}
        bt
        )


