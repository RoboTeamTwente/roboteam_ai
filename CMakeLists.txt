cmake_minimum_required(VERSION 3.10)
project(roboteam_ai)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Werror -g -O0")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")

add_subdirectory(src)
# QT settings
set(CMAKE_INCLUDE_CURRENT_DIR ON) #Find includes in corresponding build directories
set(CMAKE_AUTOMOC ON) #Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOUIC OFF) #Create code from a list of Qt designer ui files

include(cotire)
include(FindSDL2)

#for MacOS X or iOS, watchOS, tvOS(since 3.10.3)
if (APPLE)
    set(Qt5Widgets_DIR "/usr/local/opt/qt5/lib/cmake/Qt5Widgets")
    set(Qt5Core_DIR "/usr/local/opt/qt/lib/cmake/Qt5Core")
    set(Qt5Gui_DIR "/usr/local/opt/qt/lib/cmake/Qt5Gui")
    SET(Qt5Charts_DIR "/usr/local/opt/qt/lib/cmake/Qt5Charts")
    find_package(Qt5Core REQUIRED)
    find_package(Qt5Gui REQUIRED)
    set(GTEST_LIB /usr/local/lib/libgtest.a /usr/local/lib/libgtest_main.a)
else (NOT APPLE)
    SET(Qt5Widgets_DIR "/usr/include/x86_64-linux-gnu/qt5/QtWidgets")
    SET(Qt5Charts_DIR "/usr/include/x86_64-linux-gnu/qt5/QtCharts")
    set(GTEST_LIB PRIVATE gtest)
endif ()

find_package(Qt5Charts REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(SDL2 REQUIRED sdl2)

if (NOT SDL2_FOUND)
    message("SDL2 not found! For linux, run 'sudo apt install libsdl2-dev'")
endif ()

set(JSON_SOURCES
        ${PROJECT_SOURCE_DIR}/src/treeinterp/TreeInterpreter.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/BTFactory.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/JsonReader.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/PropertiesParser.cpp
        ${PROJECT_SOURCE_DIR}/src/Switches.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/OffensiveStrategy.cpp
        )

set(BT_SOURCES
        ${PROJECT_SOURCE_DIR}/src/treeinterp/Leaf.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/MidFieldHarassRole.cpp
        ${PROJECT_SOURCE_DIR}/src/treehelpers/BeingPassedToHelper.cpp
        ${PROJECT_SOURCE_DIR}/src/treehelpers/RobotOutOfFieldHelper.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/PassRole.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/SideAttackerRole.cpp
        ${PROJECT_SOURCE_DIR}/src/treeinterp/tactics/DefaultTactic.cpp
        )

set(ANALYSIS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/analysis/GameAnalyzer.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/AnalysisReport.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/RobotDanger.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/DecisionMaker.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/PlayChecker.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/play-utilities/invariants/BallBelongsToUsInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/play-utilities/Play.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/play-utilities/invariants/BallOnOurSideInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/play-utilities/invariants/AlwaysFalseInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/play-utilities/invariants/AlwaysTrueInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/analysis/play-utilities/PlayDecider.cpp
        )

set(UTILS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/utilities/GameStateManager.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/RobotDealer.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Constants.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Pause.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/RefGameState.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/IOManager.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Settings.cpp
        )

set(MANUAL_SOURCES
        ${PROJECT_SOURCE_DIR}/src/manual/JoystickManager.cpp
        ${PROJECT_SOURCE_DIR}/src/manual/JoystickHandler.cpp
        )

set(SKILLS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/skills/Skill.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/gotopos/GoToPos.cpp
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
        ${PROJECT_SOURCE_DIR}/src/skills/KickTo.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/formations/StopFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/skills/formations/BallPlacementFormation.cpp
        )

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
        ${PROJECT_SOURCE_DIR}/src/conditions/ResumePlayAfterPenalty.cpp
        )

set(CONTROL_SOURCES
        ${PROJECT_SOURCE_DIR}/src/control/ControlUtils.cpp
        ${PROJECT_SOURCE_DIR}/src/control/BasicPosControl.cpp
        ${PROJECT_SOURCE_DIR}/src/control/PosController.cpp
        ${PROJECT_SOURCE_DIR}/src/control/numtrees/NumTreePosControl.cpp
        ${PROJECT_SOURCE_DIR}/src/control/numtrees/PathPoint.cpp
        ${PROJECT_SOURCE_DIR}/src/control/numtrees/Collision.cpp
        ${PROJECT_SOURCE_DIR}/src/control/shot-controllers/ShotController.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ball-handling/BallHandlePosControl.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ball-handling/DribbleBackwards.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ball-handling/DribbleForwards.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ball-handling/RotateAroundBall.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ball-handling/RotateWithBall.cpp
        )

set(TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/test/main.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/ControlUtilsTest.cpp
        ${PROJECT_SOURCE_DIR}/test/btTests/RoleDividerTests.cpp
        ${PROJECT_SOURCE_DIR}/test/TreeInterpreterTests/BtTest.cpp
        ${PROJECT_SOURCE_DIR}/test/TreeInterpreterTests/BTFactoryTest.cpp
        ${PROJECT_SOURCE_DIR}/test/TreeInterpreterTests/BTBlackBoardTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/PropertiesParserTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/JsonTest.cpp
        ${PROJECT_SOURCE_DIR}/test/TacticTests/DefaultTacticTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/RefereeTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/RobotDealerTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/StrategyManagerTest.cpp
        ${PROJECT_SOURCE_DIR}/test/helpers/WorldHelper.h
        ${PROJECT_SOURCE_DIR}/test/helpers/WorldHelper.cpp
        ${PROJECT_SOURCE_DIR}/test/helpers/FieldHelper.h
        ${PROJECT_SOURCE_DIR}/test/helpers/FieldHelper.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/BallTests.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/RobotTests.cpp
        )

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
        ${PROJECT_SOURCE_DIR}/src/interface/api/Toggles.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/RuleSetWidget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/GraphWidget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/SettingsWidget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/ManualControlWidget.cpp

        #QT wants to know about these headers
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/PidBox.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/PidsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/MainControlsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/mainWindow.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/RobotsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/RuleSetWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/TreeVisualizerWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/VisualizationSettingsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/widget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/SettingsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/ManualControlWidget.h
        )

set(WORLD_SOURCES
        ${PROJECT_SOURCE_DIR}/src/world/Robot.cpp
        ${PROJECT_SOURCE_DIR}/src/world/Ball.cpp
        ${PROJECT_SOURCE_DIR}/src/world/World.cpp
        ${PROJECT_SOURCE_DIR}/src/world/FieldComputations.cpp
        ${PROJECT_SOURCE_DIR}/src/world/History.cpp
        ${PROJECT_SOURCE_DIR}/src/world/FutureWorld.cpp
        ${PROJECT_SOURCE_DIR}/src/world/BallPossession.cpp
        ${PROJECT_SOURCE_DIR}/src/world/Field.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/Ball.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/Robot.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/World.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/WorldData.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/RobotControllers.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/views/WorldDataView.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/views/RobotView.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/views/BallView.cpp
        )

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
        ${PROJECT_SOURCE_DIR}/src/coach/midfield/MidFieldCoach.cpp
        ${PROJECT_SOURCE_DIR}/src/coach/GetBallCoach.cpp)

# Putting it all together so we can use it in both the main executable and the test executable
set(AI_SOURCES
        src/ApplicationManager.cpp
        include/roboteam_ai/ApplicationManager.h
        ${UTILS_SOURCES}
        ${JSON_SOURCES}
        ${BT_SOURCES}
        ${ANALYSIS_SOURCES}
        ${SKILLS_SOURCES}
        ${CONDITIONS_SOURCES}
        ${CONTROL_SOURCES}
        ${TACTIC_SOURCES}
        ${COACH_SOURCES}
        ${INTERFACE_SOURCES}
        ${WORLD_SOURCES}
        ${MANUAL_SOURCES}
        )

set(AI_LIBS
        PRIVATE Qt5::Widgets
        PRIVATE Qt5::Gui
        PRIVATE Qt5::Charts
        PRIVATE roboteam_utils
        PRIVATE roboteam_proto
        PRIVATE bt
        PRIVATE ${SDL2_LIBRARY}
        )

set(AI_INCLUDES
        INTERFACE include
        PRIVATE include/roboteam_ai
        PRIVATE ${SDL2_INCLUDE_DIRS}
        PRIVATE ${SDL2IMAGE_INCLUDE_DIRS}
        )

# Main Executable
add_executable(roboteam_ai src/roboteam_ai.cpp ${AI_SOURCES})
target_link_libraries(roboteam_ai ${AI_LIBS})
target_include_directories(roboteam_ai ${AI_INCLUDES})

# Testing #
add_executable(ai_tests ${AI_SOURCES} ${TEST_SOURCES})
target_link_libraries(ai_tests ${AI_LIBS} ${GTEST_LIB})
target_include_directories(ai_tests ${AI_INCLUDES})