
        cmake_minimum_required(VERSION 3.1)
project(roboteam_ai)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O0 -Wall --coverage")
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

find_package(SDL2)
find_package(SDL2TTF)
include_directories(${SDL2_INCLUDE_DIR} ${SDL2TTF_INCLUDE_DIR})

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

### SOURCES BEGIN ###


set(JSON_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/json.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/TreeInterpreter.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/TreeInterpreter.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/BTFactory.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/BTFactory.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/JsonReader.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/JsonReader.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/PropertiesParser.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/PropertiesParser.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/Switches.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/Switches.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/treeinterp/BTImport.h)

set(DANGER_FINDER_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/CanShootModule.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/CanShootModule.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/DangerData.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/DangerData.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/DangerFinder.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/DangerFinder.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/PartialResult.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/PartialResult.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/DangerModule.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/DangerModule.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/DistanceModule.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/DistanceModule.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/FreeModule.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/FreeModule.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/HasBallModule.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/HasBallModule.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/OrientationModule.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/OrientationModule.h
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
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Blackboard.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Composite.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Composite.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Decorator.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Decorator.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Leaf.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Leaf.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Node.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Node.cpp

        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Role.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Role.h
        )

set(TACTIC_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Tactic.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Tactic.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/DemoTactic.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/DemoTactic.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/ParallelSequenceTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/ParallelSequenceTest.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/VictoryDanceTactic.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/VictoryDanceTactic.h
        )

set(IO_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/IOManager.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/IOManager.cpp
        )

set(UTILS_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/World.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/World.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Referee.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Referee.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/RobotDealer.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/RobotDealer.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Constants.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Field.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Field.cpp
        )

set(SKILLS_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Skill.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Skill.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/DefaultSkill.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/DefaultSkill.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Kick.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Kick.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Chip.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Chip.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Rotate.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Rotate.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/GoToPos.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/GoToPos.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/TurnGeneva.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/TurnGeneva.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/GoToPosLuTh.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/GoToPosLuTh.cpp
        )

set(CONDITIONS_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/HasBall.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/HasBall.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsInDefenseArea.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsInDefenseArea.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/Condition.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/Condition.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsInZone.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsInZone.cpp
        roboteam_ai/src/utilities/StrategyManager.cpp
        roboteam_ai/src/utilities/StrategyManager.h)

### SOURCES END ###

### TEST SOURCES BEGIN ###

set(SKILL_TESTS
        roboteam_ai/test/SkillTests/KickTest.cpp
        )

set(CONDITION_TESTS
        roboteam_ai/test/ConditionTests/IsInDefenseAreaTest.cpp
        roboteam_ai/test/ConditionTests/IHaveBallTest.cpp
        roboteam_ai/test/ConditionTests/IHaveBallTest.cpp
        roboteam_ai/test/ConditionTests/IsInZoneTest.cpp
        )

set(CONTROL_SOURCES
        roboteam_ai/src/control/ControlUtils.cpp
        roboteam_ai/src/control/ControlUtils.h)

### TEST SOURCES END ###

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# StrategyNode
add_executable(Roboteam_AI
        roboteam_ai/src/roboteam_ai.cpp
        ${DANGER_FINDER_SOURCES}
        ${IO_SOURCES}
        ${UTILS_SOURCES}
        ${JSON_SOURCES}
        ${BT_SOURCES}
        ${SKILLS_SOURCES}
        ${CONDITIONS_SOURCES}
        ${CONTROL_SOURCES}
        ${TACTIC_SOURCES}
        roboteam_ai/src/interface/Interface.cpp
        roboteam_ai/src/interface/Interface.h)

add_dependencies(Roboteam_AI ${catkin_EXPORTED_TARGETS})
target_link_libraries(Roboteam_AI ${catkin_LIBRARIES} ${SDL2_LIBRARY} ${SDL2TTF_LIBRARY})

#############
## Testing ##
#############
catkin_add_gtest(ai_tests
        roboteam_ai/test/main.cpp
        roboteam_ai/test/BtTest.cpp
        roboteam_ai/test/JsonTest.cpp
        roboteam_ai/test/BTFactoryTest.cpp
        roboteam_ai/test/WorldTest.cpp
        roboteam_ai/test/FieldTest.cpp
        roboteam_ai/test/DangerFinderTest.cpp
        roboteam_ai/test/BTBlackBoardTest.cpp
        roboteam_ai/test/IOTest.cpp
        roboteam_ai/test/RobotDealerTest.cpp
        roboteam_ai/test/RefereeTest.cpp
        roboteam_ai/test/StrategyManagerTest.cpp
        roboteam_ai/test/PropertiesParserTest.cpp
        ${SKILL_TESTS}
        ${CONDITION_TESTS}

        ${DANGER_FINDER_SOURCES}
        ${UTILS_SOURCES}
        ${JSON_SOURCES}
        ${BT_SOURCES}
        ${IO_SOURCES}
        ${CONDITIONS_SOURCES}
        ${SKILLS_SOURCES}
        ${CONTROL_SOURCES}
        ${TACTIC_SOURCES}
        )

target_compile_options(ai_tests PUBLIC --coverage)

target_link_libraries(ai_tests
        ${catkin_LIBRARIES}
        gcov
        ${SDL2_LIBRARY}
        ${SDL2TTF_LIBRARY}
        )

include(CodeCoverage)
APPEND_COVERAGE_COMPILER_FLAGS()
SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${PROJECT_NAME}_coverage EXECUTABLE ai_tests )
