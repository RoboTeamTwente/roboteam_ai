cmake_minimum_required(VERSION 3.4)
project(roboteam_ai)

set(CMAKE_CXX_FLAGS_DEBUG "-std=c++17 ${CMAKE_CXX_FLAGS} -O1")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 ${CMAKE_CXX_FLAGS} -Werror -O1")

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(cotire)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC OFF)


# for MacOS X or iOS, watchOS, tvOS (since 3.10.3)
if(APPLE)
    #Qt5Test_DIR=/usr/local/opt/qt/lib/cmake/Qt5Test
    #Qt5Core_DIR=/usr/local/opt/qt/lib/cmake/Qt5Core
    #Qt5_DIR=/usr/local/opt/qt/lib/cmake
    #Qt5Concurrent_DIR=/usr/local/opt/qt/lib/cmake/Qt5Concurrent
    #Qt5Gui_DIR=/usr/local/opt/qt/lib/cmake/Qt5Gui
#    set(Qt5Gui_DIR "/usr/local/opt/qt/lib/cmake/Qt5Gui")
#    set(Qt5_DIR "/usr/local/opt/qt/lib/cmake")
    set(Qt5Widgets_DIR "/usr/local/opt/qt5/lib/cmake/Qt5Widgets")

    set(Qt5Core_DIR "/usr/local/opt/qt/lib/cmake/Qt5Core")
    set(Qt5Gui_DIR "/usr/local/opt/qt/lib/cmake/Qt5Gui")

    find_package(Qt5Core REQUIRED)
    find_package(Qt5Gui REQUIRED)
endif()

# for Linux, BSD, Solaris, Minix
if(UNIX AND NOT APPLE)
    SET(Qt5Widgets_DIR "/usr/include/x86_64-linux-gnu/qt5/QtWidgets")
endif()


find_package(Qt5Widgets REQUIRED)

set(JSON_SOURCES
        ${PROJECT_SOURCE_DIR}/src/treeinterp/TreeInterpreter.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/BTFactory.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/JsonReader.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/PropertiesParser.cpp
        ${PROJECT_SOURCE_DIR}/src/Switches.cpp)

set(BT_SOURCES
        ${PROJECT_SOURCE_DIR}/src/bt/composites/MemSelector.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/composites/MemSequence.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/composites/ParallelSequence.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/composites/MemParallelSequence.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/composites/Selector.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/composites/Sequence.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/decorators/Failer.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/decorators/Inverter.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/decorators/Repeater.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/decorators/Succeeder.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/decorators/UntilFail.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/decorators/UntilSuccess.cpp

        ${PROJECT_SOURCE_DIR}/src/bt/BehaviorTree.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/Blackboard.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/Blackboard.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/Composite.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/Decorator.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/Leaf.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/Node.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/RoleDivider.cpp
        ${PROJECT_SOURCE_DIR}/src/bt/Role.cpp
        )

set(ANALYSIS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/analysis/GameAnalyzer.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/AnalysisReport.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/RobotDanger.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/DecisionMaker.cpp)

set(TACTIC_SOURCES ${PROJECT_SOURCE_DIR}/src/bt/tactics/DefaultTactic.cpp)

set(IO_SOURCES
        ${PROJECT_SOURCE_DIR}/src/io/IOManager.cpp)

set(DEMO_SOURCES ${PROJECT_SOURCE_DIR}/src/demo/JoystickDemo.cpp)

set(UTILS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/utilities/GameStateManager.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/RobotDealer.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Constants.cpp
        ${PROJECT_SOURCE_DIR}/src/control/Hungarian.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Pause.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/RefGameState.cpp)


set(SKILLS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/skills/Skill.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/gotopos/GoToPos.cpp
#        ${PROJECT_SOURCE_DIR}/src/skills/TurnGeneva.h
#        ${PROJECT_SOURCE_DIR}/src/skills/TurnGeneva.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/gotopos/SkillGoToPos.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/Dribble.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/RotateToAngle.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/GetBall.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/Pass.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/Receive.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/Harass.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/Keeper.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/InterceptBall.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/Attack.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/DribbleRotate.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/DribbleForward.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/gotopos/GTPSpecial.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/gotopos/GTPWithBall.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/gotopos/GoAroundPos.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/AvoidBall.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/gotopos/GoBehindBall.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/PreparePenalty.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/ShootPenalty.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/ShootFreeKick.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/DemoAttack.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/FreeKickPass.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/MidFieldHarasser.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/SideAttacker.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/CoachDefend.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/SlingShot.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/InterceptRobot.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/PenaltyHelper.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/ActiveStop.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/ReflectKick.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/ChipForward.cpp

        ${PROJECT_SOURCE_DIR}/src/skills/formations/Formation.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/formations/TimeoutFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/formations/KickOffUsFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/formations/KickOffThemFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/formations/PenaltyFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/formations/PenaltyFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/PenaltyKeeper.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/formations/PenaltyFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/formations/FreeKickFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/formations/DefendFreeKick.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/ballPlacement/BallPlacementReceive.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/ballPlacement/BallPlacementPass.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/DriveWithInterface.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/BallPlacementWithInterface.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/Wait.cpp
        src/skills/formations/StopFormation.cpp
        src/skills/formations/BallPlacementFormation.cpp)

set(CONDITIONS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/conditions/HasBall.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/IsInDefenseArea.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/IsRobotClosestToBall.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/CanReflectKick.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/Condition.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/StrategyManager.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/TheyHaveBall.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/WeHaveBall.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/BallKickedToOurGoal.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/BallInDefenseAreaAndStill.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/IsBallOnOurSide.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/IsBeingPassedTo.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/IsCloseToPoint.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/BallOutOfField.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/IsBallCloseToBorder.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/BallNearOurGoalLineAndStill.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/HasClearShot.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/TwoRobotBallPlacement.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/ShouldHandleBall.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/IsOnPassLine.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/BallIsClose.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/CanPlay.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/RobotOutside.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/RefStateIsNormalPlay.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/RefBallIsMoving.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/ResumePlayAfterPenalty.cpp)

set(CONTROL_SOURCES
        ${PROJECT_SOURCE_DIR}/src/control/ControlUtils.cpp
        ${PROJECT_SOURCE_DIR}/src/control/pid.cpp

        # PositionControllers
        ${PROJECT_SOURCE_DIR}/src/control/BasicPosControl.cpp
        ${PROJECT_SOURCE_DIR}/src/control/PosController.cpp

        # NumTrees
        ${PROJECT_SOURCE_DIR}/src/control/numTrees/NumTreePosControl.cpp
        ${PROJECT_SOURCE_DIR}/src/control/numTrees/PathPoint.cpp
        ${PROJECT_SOURCE_DIR}/src/control/numTrees/Collision.cpp

        # ShotController
        ${PROJECT_SOURCE_DIR}/src/control/shotControllers/ShotController.cpp

        # BallHandling
        ${PROJECT_SOURCE_DIR}/src/control/ballHandling/BallHandlePosControl.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ballHandling/DribbleBackwards.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ballHandling/DribbleForwards.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ballHandling/RotateAroundBall.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ballHandling/RotateWithBall.cpp
        )

set(TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/test/main.cpp
        ${PROJECT_SOURCE_DIR}/test/TreeInterpreterTests/BtTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/JsonTest.cpp
        ${PROJECT_SOURCE_DIR}/test/TreeInterpreterTests/BTFactoryTest.cpp
        #${PROJECT_SOURCE_DIR}/test/WorldTests/WorldTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/FieldTest.cpp
        ${PROJECT_SOURCE_DIR}/test/TreeInterpreterTests/BTBlackBoardTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/IOTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/RefereeTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/StrategyManagerTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/PropertiesParserTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/ControlUtilsTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/TheyHaveBallTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/WeHaveBallTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/HasBallTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/IsInDefenseAreaTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/IsRobotClosestToBallTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/IsBeingPassedToTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/BallInDefenseAreaAndStillTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/BallKickedToOurGoalTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/CanReflectKickTest.cpp
        ${PROJECT_SOURCE_DIR}/test/SkillTests/GoToPosTest.cpp
        ${PROJECT_SOURCE_DIR}/test/TacticTests/DefaultTacticTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/RobotDealerTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/HungarianTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/BallPossessionTests.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/HistoryTests.cpp
        ${PROJECT_SOURCE_DIR}/test/InterfaceTests/InterfaceTest.cpp
        ${PROJECT_SOURCE_DIR}/test/InterfaceTests/MainWindowTest.cpp
        ${PROJECT_SOURCE_DIR}/test/InterfaceTests/TreeVisualizerTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/ControlUtilsTest.cpp
        ${PROJECT_SOURCE_DIR}/test/SkillTests/PassTest.cpp
        ${PROJECT_SOURCE_DIR}/test/helpers/WorldHelper.h
        ${PROJECT_SOURCE_DIR}/test/helpers/WorldHelper.cpp
        ${PROJECT_SOURCE_DIR}/test/helpers/FieldHelper.h
        ${PROJECT_SOURCE_DIR}/test/helpers/FieldHelper.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/BallOutOfFieldtest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/IsBallOnOurSideTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ConditionTests/IsBallCloseToBorderTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/JoyStickTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/PositionUtilsTest.cpp
        ${PROJECT_SOURCE_DIR}/test/CoachTests/BallPlacementCoachTest.cpp
        ${PROJECT_SOURCE_DIR}/test/CoachTests/OffensiveCoachTest.cpp
        ${PROJECT_SOURCE_DIR}/test/SkillTests/FormationTest.cpp
        ${PROJECT_SOURCE_DIR}/test/CoachTests/BallPlacementCoachTest.cpp
        ${PROJECT_SOURCE_DIR}/test/CoachTests/DefensiveCoachTest.cpp
        ${PROJECT_SOURCE_DIR}/test/CoachTests/OffensiveCoachTest.cpp
        ${PROJECT_SOURCE_DIR}/test/AnalysisTests/DecisionmakerTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/ShotControllerTest.cpp
        ${PROJECT_SOURCE_DIR}/test/InterfaceTests/MainControlsWidgetTest.cpp
        ${PROJECT_SOURCE_DIR}/test/btTests/RoleDividerTests.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/PosControllerTests/NumTreePosControlTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/PosControllerTests/RobotCommandTest.cpp ../main.cpp)

set(INTERFACE_SOURCES
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/mainWindow.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/widget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/api/Input.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/api/Output.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/TreeVisualizerWidget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/RobotsWidget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/PidBox.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/PidsWidget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/MainControlsWidget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/VisualizationSettingsWidget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/CheckboxWidget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/api/Toggles.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/RuleSetWidget.cpp)

set(WORLD_SOURCES
        ${PROJECT_SOURCE_DIR}/src/world/Robot.cpp
        ${PROJECT_SOURCE_DIR}/src/world/Ball.cpp

        ${PROJECT_SOURCE_DIR}/src/world/World.cpp
        ${PROJECT_SOURCE_DIR}/src/world/Field.cpp
        ${PROJECT_SOURCE_DIR}/src/world/WorldManager.cpp
        ${PROJECT_SOURCE_DIR}/src/world/History.cpp
        ${PROJECT_SOURCE_DIR}/src/world/FutureWorld.cpp
        ${PROJECT_SOURCE_DIR}/src/world/BallPossession.cpp)

set(COACH_SOURCES
        ${PROJECT_SOURCE_DIR}/src/coach/heuristics/CoachHeuristics.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/heuristics/PassScore.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/heuristics/OffensiveScore.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/PassCoach.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/BallplacementCoach.cpp
        ${PROJECT_SOURCE_DIR}/src/control/PositionUtils.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/OffensiveCoach.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/defence/DefencePositionCoach.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/defence/DefenceDealer.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/defence/PossiblePass.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/midField/MidFieldCoach.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/GetBallCoach.cpp)

# Main Executable
add_library(roboteam_ai

        src/roboteam_ai.cpp
        src/ApplicationManager.cpp
        include/roboteam_ai/ApplicationManager.h
        ${IO_SOURCES}
        ${UTILS_SOURCES}
        ${JSON_SOURCES}
        ${BT_SOURCES}
        ${ANALYSIS_SOURCES}
        ${SKILLS_SOURCES}
        ${CONDITIONS_SOURCES}
        ${CONTROL_SOURCES}
        ${TACTIC_SOURCES}
        ${COACH_SOURCES}
        ${DEMO_SOURCES}
        ${INTERFACE_SOURCES}
        ${WORLD_SOURCES} src/AI.cpp include/roboteam_ai/AI.h

        # QT wants to know about these headers
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/PidBox.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/PidsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/MainControlsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/mainWindow.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/RobotsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/CheckboxWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/RuleSetWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/TreeVisualizerWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/VisualizationSettingsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/widget.h

        )

target_link_libraries(roboteam_ai Qt5::Widgets Qt5::Gui roboteam_utils)

target_include_directories(roboteam_ai PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src )

#############
## Testing ##
#############
add_executable(ai_tests
        ${UTILS_SOURCES}
        ${JSON_SOURCES}
        ${BT_SOURCES}
        ${IO_SOURCES}
        ${CONDITIONS_SOURCES}
        ${SKILLS_SOURCES}
        ${CONTROL_SOURCES}
        ${ANALYSIS_SOURCES}
        ${TACTIC_SOURCES}
        ${DEMO_SOURCES}
        ${COACH_SOURCES}
        ${INTERFACE_SOURCES}
        ${TEST_SOURCES}
        ${WORLD_SOURCES})

target_link_libraries(ai_tests
        roboteam_utils
        gcov
        Qt5::Widgets Qt5::Gui)


include(CodeCoverage)
APPEND_COVERAGE_COMPILER_FLAGS()
SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${PROJECT_NAME}_coverage EXECUTABLE ai_tests)

# required properties for cotire
set_property(TARGET roboteam_ai PROPERTY CXX_STANDARD 11)
set_property(TARGET roboteam_ai PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ai_tests PROPERTY CXX_STANDARD 11)
set_property(TARGET ai_tests PROPERTY CXX_STANDARD_REQUIRED ON)


if(UNIX AND NOT APPLE)
    cotire(roboteam_ai)
    cotire(ai_tests)
endif()

