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
set(JSON_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/json.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/TreeInterpreter.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/TreeInterpreter.cpp
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(roboteam_ai/src)

add_dependencies(bt
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

# StrategyNode
add_executable(StrategyNode roboteam_ai/src/StrategyNode.cpp roboteam_ai/src/io/IOManager.h roboteam_ai/src/io/IOManager.cpp roboteam_ai/src/StrategyNode.cpp roboteam_ai/src/io/StrategyIOManager.cpp roboteam_ai/src/io/StrategyIOManager.h roboteam_ai/src/io/RoleIOManager.cpp roboteam_ai/src/io/RoleIOManager.h )

add_executable(Json roboteam_ai/src/treeinterp/TreeInterpreter.cpp
        roboteam_ai/src/treeinterp/TreeInterpreter.h roboteam_ai/src/treeinterp/json.h
        roboteam_ai/src/treeinterp/BTFactory.cpp roboteam_ai/src/treeinterp/BTFactory.h)


add_dependencies(StrategyNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(StrategyNode ${catkin_LIBRARIES} bt)


#############
## Testing ##
#############
catkin_add_gtest(bt_test
        roboteam_ai/test/main.cpp
        roboteam_ai/test/BtTest.cpp
        roboteam_ai/test/JsonTest.cpp
        ${JSON_SOURCES}

        )

target_link_libraries(bt_test
        ${catkin_LIBRARIES}
        bt
        )