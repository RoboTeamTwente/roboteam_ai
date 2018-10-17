cmake_minimum_required(VERSION 3.1)
project(roboteam_ai)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O0 -Wall --coverage")

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

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
set(JSON_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/json.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/TreeInterpreter.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/TreeInterpreter.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/BTFactory.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/BTFactory.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/JsonReader.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/JsonReader.h
        roboteam_ai/src/treeinterp/BTImport.h)


set(DANGER_FINDER_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/CanShootModule.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/modules/CanShootModule.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/DangerData.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/DangerData.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/DangerFinder.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/DangerFinder.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/PartialResult.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/DangerFinder/PartialResult.h
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
        )


set(BT_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/MemSelector.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/MemSelector.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/MemSequence.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/MemSequence.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/ParallelSequence.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/ParallelSequence.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/Selector.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/Selector.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/Sequence.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/Sequence.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/Failer.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/Failer.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/Inverter.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/Inverter.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/Repeater.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/Repeater.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/Succeeder.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/Succeeder.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/UntilFail.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/UntilFail.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/UntilSuccess.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/decorators/UntilSuccess.cpp
                                          
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/BehaviorTree.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/BehaviorTree.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Blackboard.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Blackboard.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Composite.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Composite.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Decorator.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Decorator.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Leaf.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Leaf.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Node.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Node.cpp
        )

set(IO_SOURCES
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/IOManager.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/IOManager.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/StrategyIOManager.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/StrategyIOManager.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/RoleIOManager.cpp
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/RoleIOManager.h)

set(UTILS_SOURCES
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/World.h
    ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/World.cpp
    )

catkin_package()


include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory( ${PROJECT_SOURCE_DIR}/roboteam_ai/src)

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# StrategyNode
add_executable(AIStrategyNode
        roboteam_ai/src/StrategyNode.cpp
        ${DANGER_FINDER_SOURCES}
        ${IO_SOURCES}
        ${UTILS_SOURCES}
        ${JSON_SOURCES}
        ${BT_SOURCES}
        )
add_dependencies(AIStrategyNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(AIStrategyNode ${catkin_LIBRARIES})

add_executable(AIRoleNode
        roboteam_ai/src/RoleNode.cpp
        ${IO_SOURCES}
        )

add_dependencies(AIRoleNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(AIRoleNode ${catkin_LIBRARIES} ai_bt)

#############
## Testing ##
#############
catkin_add_gtest(ai_tests
        roboteam_ai/test/main.cpp
        roboteam_ai/test/BtTest.cpp
        roboteam_ai/test/JsonTest.cpp
        roboteam_ai/test/BTFactoryTest.cpp
        roboteam_ai/test/WorldTest.cpp
        roboteam_ai/test/DangerFinderTest.cpp
        roboteam_ai/test/BTBlackBoardTest.cpp
        ${DANGER_FINDER_SOURCES}
        ${UTILS_SOURCES}
        ${JSON_SOURCES}
        ${BT_SOURCES}
        )

target_compile_options(ai_tests PUBLIC --coverage)

target_link_libraries(ai_tests
        ${catkin_LIBRARIES}
        gcov
        )

include(CodeCoverage)
APPEND_COVERAGE_COMPILER_FLAGS()
SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${PROJECT_NAME}_coverage EXECUTABLE ai_tests )




