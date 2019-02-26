cmake_minimum_required(VERSION 3.1)
project(roboteam_ai)

#SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -O0 -Wall --coverage")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++11 ${CMAKE_CXX_FLAGS} -g3 -O0 -Wall --coverage")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 ${CMAKE_CXX_FLAGS} -Werror -O1")

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

SET(Qt5Widgets_DIR "/usr/include/x86_64-linux-gnu/qt5/QtWidgets")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roboteam_msgs
        roboteam_utils
        roscpp
        roslib
        std_msgs
        actionlib_msgs
        actionlib
        uuid_msgs
        unique_id
        roslaunch)

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
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/dangerfinder/modules/OrientationModule.h)

set(BT_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/MemSelector.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/MemSelector.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/MemSequence.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/MemSequence.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/ParallelSequence.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/ParallelSequence.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/MemParallelSequence.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/composites/MemParallelSequence.cpp
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
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/EnterFormationTactic.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/EnterFormationTactic.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/AvoidBallTactic.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/AvoidBallTactic.h)

set(TACTIC_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Tactic.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/Tactic.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/VictoryDanceTactic.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/VictoryDanceTactic.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/DefaultTactic.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/bt/tactics/DefaultTactic.h)

set(IO_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/IOManager.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/IOManager.cpp)

set(DEMO_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/demo/JoystickDemo.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/demo/JoystickDemo.h)

set(UTILS_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/World.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/World.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Referee.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Referee.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/RobotDealer.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/RobotDealer.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Constants.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Constants.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Field.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Field.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Coach.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Coach.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/ApplicationManager.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/ApplicationManager.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Hungarian.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Hungarian.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Pause.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Pause.h
        )


set(SKILLS_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Skill.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Skill.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Kick.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Kick.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Chip.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Chip.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/GoToPos.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/GoToPos.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/TurnGeneva.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/TurnGeneva.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/SkillGoToPos.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/SkillGoToPos.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Dribble.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Dribble.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/RotateToAngle.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/RotateToAngle.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/GetBall.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/GetBall.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Pass.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Pass.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Receive.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Receive.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/DefendOnRobot.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/DefendOnRobot.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Halt.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Halt.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Harass.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Harass.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Keeper.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Keeper.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/InterceptBall.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/InterceptBall.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Attack.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Attack.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/DribbleRotate.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/DribbleRotate.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/BasicGoToPos.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/BasicGoToPos.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/GoAroundPos.cpp 
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/GoAroundPos.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Defend.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/Defend.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/EnterFormation.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/EnterFormation.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/AvoidBall.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/AvoidBall.h)

set(CONDITIONS_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/HasBall.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/HasBall.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsInDefenseArea.hpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsInDefenseArea.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsRobotClosestToBall.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsRobotClosestToBall.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/Condition.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/Condition.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/StrategyManager.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/StrategyManager.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/TheyHaveBall.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/TheyHaveBall.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/WeHaveBall.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/WeHaveBall.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/BallKickedToOurGoal.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/BallKickedToOurGoal.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/BallInDefenseAreaAndStill.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/BallInDefenseAreaAndStill.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsBallOnOurSide.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsBallOnOurSide.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsBeingPassedTo.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsBeingPassedTo.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsCloseToPoint.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/IsCloseToPoint.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/BallOutOfField.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/conditions/BallOutOfField.h)

set(CONTROL_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ControlUtils.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ControlUtils.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/positionControllers/ControlGoToPosBallControl.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/positionControllers/ControlGoToPosBallControl.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/PositionController.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/PositionController.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/PIDController.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/PIDController.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/positionControllers/NumTreePosControl.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/positionControllers/NumTreePosControl.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/positionControllers/PosVelAngle.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/positionControllers/PositionControlIncludes.h)


set(TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/main.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/TreeInterpreterTests/BtTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/JsonTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/TreeInterpreterTests/BTFactoryTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/WorldTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/FieldTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/DangerFinderTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/TreeInterpreterTests/BTBlackBoardTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/IOTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/RefereeTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/StrategyManagerTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/PropertiesParserTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ControlTests/ControlUtilsTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ApplicationManagerTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ConditionTests/TheyHaveBallTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ConditionTests/WeHaveBallTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ConditionTests/HasBallTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ConditionTests/IsInDefenseAreaTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ConditionTests/IsRobotClosestToBallTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ConditionTests/BallInDefenseAreaAndStillTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ConditionTests/BallKickedToOurGoalTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/SkillTests/KickTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/SkillTests/GoToPosTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/TacticTests/DefaultTacticTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ControlTests/ControllerTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/RobotDealerTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/CoachTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/HungarianTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/InterfaceTests/InterfaceTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/InterfaceTests/VisualizerTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/InterfaceTests/MainWindowTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/InterfaceTests/TreeVisualizerTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ControlTests/ControlUtilsTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/SkillTests/DefendOnRobotTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/SkillTests/PassTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/helpers/WorldHelper.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/helpers/WorldHelper.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ConditionTests/BallOutOfFieldtest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/ConditionTests/IsBallOnOurSideTest.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/test/UtilTests/JoyStickTest.cpp)

set(INTERFACE_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/mainWindow.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/mainWindow.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widget.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widget.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/drawer.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/drawer.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/InterfaceValues.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/InterfaceValues.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/TreeVisualizerWidget.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/TreeVisualizerWidget.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/RobotsWidget.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/RobotsWidget.h)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# copy all jsons to devel of RO -m "fix S
file(GLOB JSONSa "${PROJECT_SOURCE_DIR}/roboteam_ai/src/jsons/keeper/*.json")
file(COPY ${JSONSa} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/roboteam_ai/src/jsons/keeper)

file(GLOB JSONSb "${PROJECT_SOURCE_DIR}/roboteam_ai/src/jsons/strategies/*.json")
file(COPY ${JSONSb} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/roboteam_ai/src/jsons/strategies)

file(GLOB JSONSc"${PROJECT_SOURCE_DIR}/roboteam_ai/src/jsons/tactics/*.json")
file(COPY ${JSONSc} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/roboteam_ai/src/jsons/tactics)

# Main Executable
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
        ${DEMO_SOURCES}
        ${INTERFACE_SOURCES})

add_dependencies(Roboteam_AI ${catkin_EXPORTED_TARGETS})
target_link_libraries(Roboteam_AI ${catkin_LIBRARIES} Qt5::Widgets Qt5::Gui)

#############
## Testing ##
#############
catkin_add_gtest(ai_tests
        ${DANGER_FINDER_SOURCES}
        ${UTILS_SOURCES}
        ${JSON_SOURCES}
        ${BT_SOURCES}
        ${IO_SOURCES}
        ${CONDITIONS_SOURCES}
        ${SKILLS_SOURCES}
        ${CONTROL_SOURCES}
        ${TACTIC_SOURCES}
        ${DEMO_SOURCES}
        ${INTERFACE_SOURCES}
        ${TEST_SOURCES}
        )

target_compile_options(ai_tests PUBLIC --coverage)

target_link_libraries(ai_tests
        ${catkin_LIBRARIES}
        gcov
        Qt5::Widgets Qt5::Gui)

include(CodeCoverage)
APPEND_COVERAGE_COMPILER_FLAGS()
SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${PROJECT_NAME}_coverage EXECUTABLE ai_tests)
