cmake_minimum_required(VERSION 3.10)
project(roboteam_ai)

add_subdirectory(src)
# QT settings
set(CMAKE_INCLUDE_CURRENT_DIR ON) #Find includes in corresponding build directories
set(CMAKE_AUTOMOC ON) #Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOUIC OFF) #Create code from a list of Qt designer ui files

#for MacOS X or iOS, watchOS, tvOS(since 3.10.3)
if (APPLE)
    set(Qt5Widgets_DIR "/usr/local/opt/qt5/lib/cmake/Qt5Widgets")
    set(Qt5Core_DIR "/usr/local/opt/qt/lib/cmake/Qt5Core")
    set(Qt5Gui_DIR "/usr/local/opt/qt/lib/cmake/Qt5Gui")
    SET(Qt5Charts_DIR "/usr/local/opt/qt/lib/cmake/Qt5Charts")
    find_package(Qt5Core REQUIRED)
    find_package(Qt5Gui REQUIRED)
    set(GTEST_LIB
            /usr/local/lib/libgtest.a
            /usr/local/lib/libgtest_main.a
            /usr/local/lib/libgmock.a
            /usr/local/lib/libgmock_main.a
            )
else (NOT APPLE)
    SET(Qt5Widgets_DIR "/usr/include/x86_64-linux-gnu/qt5/QtWidgets")
    SET(Qt5Charts_DIR "/usr/include/x86_64-linux-gnu/qt5/QtCharts")
    set(GTEST_LIB
            PRIVATE gtest
            PRIVATE gmock)
endif ()

find_package(Qt5Charts REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(UTILS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/utilities/GameStateManager.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Constants.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Pause.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/RefGameState.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Dealer.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/IOManager.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Settings.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/StrategyManager.cpp
        )

set(MANUAL_SOURCES
        ${PROJECT_SOURCE_DIR}/src/manual/JoystickManager.cpp
        ${PROJECT_SOURCE_DIR}/src/manual/JoystickHandler.cpp
        )

set(SKILLS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Skill.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/Kick.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/Chip.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/Rotate.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/GoToPos.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/Shoot.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/Orbit.cpp
        )

set(TACTICS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Tactic.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/TestTactic.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/DriveWithBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/KickAtPos.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/GetBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/BlockRobot.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/Receive.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/ChipAtPos.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/ShootAtPos.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/Intercept.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/Formation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/GetBallInDirection.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/BlockBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/KeeperBlockBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/AvoidBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/PositionAndAim.cpp

        ${PROJECT_SOURCE_DIR}/src/stp/constants/ControlConstants.cpp
        )

set(ROLES_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Role.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/TestRole.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/Attacker.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/PassReceiver.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/Passer.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/Defender.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/Formation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/Harasser.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/Keeper.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/BallPlacer.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/BallAvoider.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/PenaltyKeeper.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/BallGetter.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/BallReflector.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/DemoKeeper.cpp
        )

set(PLAYS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Play.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/PlayChecker.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/PlayDecider.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/TestPlay.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/AttackingPass.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/Attack.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/DefendShot.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/DefendPass.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/DefensiveStopFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/AggressiveStopFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/BallPlacementUs.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/BallPlacementThem.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/TimeOut.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/PenaltyThemPrepare.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/PenaltyUsPrepare.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/PenaltyThem.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/PenaltyUs.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/KickOffUsPrepare.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/KickOffThemPrepare.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/KickOffThem.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/KickOffUs.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/GetBallPossession.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/FreeKickThem.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/GetBallRisky.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/ReflectKick.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/GenericPass.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/PenaltyDemo.cpp
        )

set(INVARIANTS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BaseInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/WeHaveBallInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallGotShotInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallMovesSlowInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallOnOurSideInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallOnTheirSideInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/FreedomOfRobotsInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallCloseToUsInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallCloseToThemInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/DistanceFromBallInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/WeHaveMajorityInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/GoalVisionInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/GoalVisionFromBallInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallIsFreeInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/FirstArrivalToBallInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallShotOrCloseToThemInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/NoGoalVisionFromBallInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallClosestToUsInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/BallPlacementUsGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/BallPlacementThemGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/HaltGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/KickOffUsGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/KickOffThemGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/KickOffUsPrepareGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/KickOffThemPrepareGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/FreeKickUsGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/FreeKickThemGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/NormalOrFreeKickUsGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/PenaltyUsGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/PenaltyThemGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/PenaltyUsPrepareGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/PenaltyThemPrepareGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/NormalPlayGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/StopGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/game_states/TimeOutGameStateInvariant.cpp
        )

set(CONTROL_SOURCES
        ${PROJECT_SOURCE_DIR}/src/control/ControlModule.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ControlUtils.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathPlanning/NumTreesPlanning.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/CollisionDetector.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/PositionControl.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathPlanning/VoronoiPathPlanning.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathTracking/DensePathTracking.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathTracking/PidTracking.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/PathPointNode.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathPlanning/PathPlanningAlgorithm.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathTracking/PathTrackingAlgorithm.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/PositionControlUtils.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/WorldObjects.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/BBTrajectory1D.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/BBTrajectory2D.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/WorldObjects.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/WorldObjects.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/WorldObjects.cpp)

set(TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/test/main.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/ControlUtilsTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/RefereeTest.cpp
        ${PROJECT_SOURCE_DIR}/test/helpers/WorldHelper.h
        ${PROJECT_SOURCE_DIR}/test/helpers/WorldHelper.cpp
        ${PROJECT_SOURCE_DIR}/test/helpers/FieldHelper.h
        ${PROJECT_SOURCE_DIR}/test/helpers/FieldHelper.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/BallTests.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/RobotTests.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/FieldComputationTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/HistorySizeTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/HistoryRetrievalTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/WhichRobotHasBallTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/WorldResetTests.cpp
        ${PROJECT_SOURCE_DIR}/test/StpTests/PlayCheckerTests.cpp
        ${PROJECT_SOURCE_DIR}/test/StpTests/PlayDeciderTests.cpp
        ${PROJECT_SOURCE_DIR}/test/StpTests/TacticTests.cpp
        test/ControlTests/BBTrajectory/BBTrajectory1DTest.cpp)


set(INTERFACE_SOURCES
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/mainWindow.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/widget.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/api/Input.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/api/Output.cpp
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/STPVisualizerWidget.cpp
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
        ${PROJECT_SOURCE_DIR}/src/interface/widgets/PlaysWidget.cpp

        #QT wants to know about these headers
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/PidBox.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/PidsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/MainControlsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/mainWindow.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/RobotsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/RuleSetWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/STPVisualizerWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/VisualizationSettingsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/widget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/SettingsWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/ManualControlWidget.h
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/PlaysWidget.hpp
        )

set(WORLD_SOURCES
        ${PROJECT_SOURCE_DIR}/src/world/FieldComputations.cpp
        ${PROJECT_SOURCE_DIR}/src/world/Field.cpp
        ${PROJECT_SOURCE_DIR}/src/world/Ball.cpp
        ${PROJECT_SOURCE_DIR}/src/world/Robot.cpp
        ${PROJECT_SOURCE_DIR}/src/world/World.cpp
        ${PROJECT_SOURCE_DIR}/src/world/WorldData.cpp
        ${PROJECT_SOURCE_DIR}/src/world/views/WorldDataView.cpp
        ${PROJECT_SOURCE_DIR}/src/world/views/RobotView.cpp
        ${PROJECT_SOURCE_DIR}/src/world/views/BallView.cpp
        )

# Putting it all together so we can use it in both the main executable and the test executable
set(AI_SOURCES
        ${PROJECT_SOURCE_DIR}/src/ApplicationManager.cpp
        ${UTILS_SOURCES}
        ${SKILLS_SOURCES}
        ${TACTICS_SOURCES}
        ${ROLES_SOURCES}
        ${PLAYS_SOURCES}
        ${INVARIANTS_SOURCES}
        ${CONTROL_SOURCES}
        ${INTERFACE_SOURCES}
        ${WORLD_SOURCES}
        ${MANUAL_SOURCES} src/stp/roles/DemoKeeper.cpp include/roboteam_ai/stp/roles/DemoKeeper.h)

set(AI_LIBS
        PRIVATE Qt5::Widgets
        PRIVATE Qt5::Gui
        PRIVATE Qt5::Charts
        PRIVATE roboteam_utils
        PRIVATE NFParam
        PRIVATE SDL2
        )

set(AI_INCLUDES
        INTERFACE include
        PRIVATE include/roboteam_ai
        )

add_library(ai_lib ${AI_SOURCES})
target_link_libraries(ai_lib ${AI_LIBS})
target_include_directories(ai_lib ${AI_INCLUDES})

# Main Executable
add_executable(roboteam_ai src/roboteam_ai.cpp)
target_link_libraries(roboteam_ai ${AI_LIBS} ai_lib)
target_include_directories(roboteam_ai ${AI_INCLUDES})

# Testing #
add_executable(ai_tests ${TEST_SOURCES})

target_link_libraries(ai_tests ${AI_LIBS} ${GTEST_LIB} ai_lib)
target_include_directories(ai_tests ${AI_INCLUDES} test)

