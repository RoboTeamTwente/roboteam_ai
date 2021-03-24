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
        )

set(TACTICS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Tactic.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/constants/ControlConstants.cpp
        # /
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/GetBehindBallInDirection.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/Intercept.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/PositionAndAim.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/KeeperBlockBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/TestTactic.cpp
        # active/
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/ChipAtPos.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/DriveWithBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/GetBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/KickAtPos.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/Receive.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/ShootAtPos.cpp
        # passive/
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/AvoidBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/BlockBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/BlockRobot.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/Formation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/Walling.cpp)

set(ROLES_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Role.cpp
        # /
        ${PROJECT_SOURCE_DIR}/src/stp/roles/TestRole.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/Keeper.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/PenaltyKeeper.cpp
        # active
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/Attacker.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/BallGetter.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/BallPlacer.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/BallReflector.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/Passer.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/PassReceiver.cpp
        # passive
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/BallAvoider.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/Defender.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/Formation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/Harasser.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/Waller.cpp)

set(PLAYS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Play.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/PlayChecker.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/PlayDecider.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/PlayEvaluator.cpp
        # /
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/TestPlay.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/ReflectKick.cpp
        # contested/
        ${PROJECT_SOURCE_DIR}/src/stp/plays/contested/GetBallPossession.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/contested/GetBallRisky.cpp
        # defensive/
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/defensive/DefendPass.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/defensive/DefendShot.cpp
        # offensive/
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/offensive/Attack.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/offensive/AttackingPass.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/offensive/GenericPass.cpp
        # referee_specific/
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/AggressiveStopFormation.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/BallPlacementThem.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/BallPlacementUs.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/DefensiveStopFormation.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/FreeKickThem.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/Halt.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/KickOffThem.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/KickOffThemPrepare.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/KickOffUs.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/KickOffUsPrepare.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/PenaltyThem.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/PenaltyThemPrepare.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/PenaltyUs.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/PenaltyUsPrepare.cpp
#        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/TimeOut.cpp
        )

set(EVALUATION_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/WeHaveBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallGotShotGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallMovesSlowGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallOnOurSideGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallOnTheirSideGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/FreedomOfRobotsGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallCloseToUsGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallCloseToThemGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/DistanceFromBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/WeHaveMajorityGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/GoalVisionGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/GoalVisionFromBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallIsFreeGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/NoGoalVisionFromBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallClosestToUsGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/BallPlacementUsGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/BallPlacementThemGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/HaltGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/KickOffUsGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/KickOffThemGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/KickOffUsPrepareGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/KickOffThemPrepareGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/FreeKickUsGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/FreeKickThemGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/NormalOrFreeKickUsGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/PenaltyUsGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/PenaltyThemGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/PenaltyUsPrepareGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/PenaltyThemPrepareGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/NormalPlayGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/StopGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/TimeOutGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/position/OpennessEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/position/LineOfSightEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/position/GoalShotEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/position/TimeToPositionEvaluation.cpp
        )

set(COMPUTATION_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/computations/PositionComputations.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/computations/GoalComputations.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/computations/PassComputations.cpp
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
        )

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
        )


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
        ${EVALUATION_SOURCES}
        ${CONTROL_SOURCES}
        ${INTERFACE_SOURCES}
        ${WORLD_SOURCES}
        ${MANUAL_SOURCES}
        ${COMPUTATION_SOURCES}
        )

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
