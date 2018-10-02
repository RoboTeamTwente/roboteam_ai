cmake_minimum_required(VERSION 3.1)
project(roboteam_ai)

find_package(catkin REQUIRED COMPONENTS
        roboteam_msgs
        roboteam_utils
        roboteam_world
        roscpp
        roslib
        std_msgs
        actionlib_msgs
        actionlib
        uuid_msgs
        unique_id
        )

set(DANGER_FINDER_SOURCES
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/CanShootModule.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/CanShootModule.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/DangerData.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/DangerData.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/DangerFinder.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/DangerFinder.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/DangerFinderConfig.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/DangerFinderConfig.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/DangerModule.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/DangerModule.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/DistanceModule.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/DistanceModule.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/FreeModule.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/FreeModule.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/HasBallModule.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/HasBallModule.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/OrientationModule.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/OrientationModule.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/WillReceiveBallModule.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/WillReceiveBallModule.h
    )

set(IO_SOURCES
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/IOManager.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/IOManager.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/StrategyIOManager.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/StrategyIOManager.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/RoleIOManager.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/RoleIOManager.h)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory( ${PROJECT_SOURCE_DIR}/roboteam_ai/src)

add_dependencies(ai_bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# StrategyNode
add_executable(AIStrategyNode
        roboteam_ai/src/StrategyNode.cpp
        ${DANGER_FINDER_SOURCES}
        ${IO_SOURCES}
        )
add_dependencies(AIStrategyNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(AIStrategyNode ${catkin_LIBRARIES} ai_bt)

add_executable(AIRoleNode
        roboteam_ai/src/RoleNode.cpp
        ${IO_SOURCES}
        )

add_dependencies(AIRoleNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(AIRoleNode ${catkin_LIBRARIES} ai_bt)

#############
## Testing ##
#############
catkin_add_gtest(ai_bt_test
        roboteam_ai/test/main.cpp
        roboteam_ai/test/BtTest.cpp
        )

target_link_libraries(ai_bt_test
        ${catkin_LIBRARIES}
        ai_bt
        )


