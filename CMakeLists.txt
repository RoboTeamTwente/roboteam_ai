cmake_minimum_required(VERSION 3.16)
project(roboteam_ai)

add_subdirectory(src)

###### UTILS #######
add_library(roboteam_ai_utils
        ${PROJECT_SOURCE_DIR}/src/utilities/GameStateManager.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Constants.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Pause.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/RefGameState.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Dealer.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/IOManager.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/Settings.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/StrategyManager.cpp
        ${PROJECT_SOURCE_DIR}/src/utilities/normalize.cpp
)
target_link_libraries(roboteam_ai_utils
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE Qt5::Widgets
)
target_include_directories(roboteam_ai_utils
        PRIVATE include/roboteam_ai
)
target_compile_options(roboteam_ai_utils PRIVATE "${COMPILER_FLAGS}")

###### MANUAL #######
add_library(roboteam_ai_manual
        ${PROJECT_SOURCE_DIR}/src/manual/JoystickManager.cpp
        ${PROJECT_SOURCE_DIR}/src/manual/JoystickHandler.cpp
)
target_include_directories(roboteam_ai_manual
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_manual
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE SDL2
        PRIVATE Qt5::Widgets
)
target_compile_options(roboteam_ai_manual PRIVATE "${COMPILER_FLAGS}")

###### SKILLS #######
add_library(roboteam_ai_skills
        ${PROJECT_SOURCE_DIR}/src/stp/Skill.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/Kick.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/Rotate.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/GoToPos.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/Orbit.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/OrbitAngular.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/skills/TestSkill.cpp
)
target_include_directories(roboteam_ai_skills
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_skills
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE Qt5::Widgets
)
target_compile_options(roboteam_ai_skills PRIVATE "${COMPILER_FLAGS}")

###### TACTICS #######
add_library(roboteam_ai_tactics
        ${PROJECT_SOURCE_DIR}/src/stp/Tactic.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/constants/ControlConstants.cpp
        # /
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/KeeperBlockBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/TestTactic.cpp
        # active/
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/DriveWithBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/GetBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/GetBehindBallInDirection.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/KickAtPos.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/OrbitKick.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/active/Receive.cpp
        # passive/
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/AvoidBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/BlockBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/BlockRobot.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/Formation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/tactics/passive/BallStandBack.cpp)
target_include_directories(roboteam_ai_tactics
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_tactics
        PRIVATE roboteam_utils
        PRIVATE roboteam_networking
        PRIVATE Qt5::Widgets
)
target_compile_options(roboteam_ai_tactics PRIVATE "${COMPILER_FLAGS}")

###### ROLES #######
add_library(roboteam_ai_roles
        ${PROJECT_SOURCE_DIR}/src/stp/Role.cpp
        # /
        ${PROJECT_SOURCE_DIR}/src/stp/roles/TestRole.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/Keeper.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/PenaltyKeeper.cpp
        # active
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/Attacker.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/Harasser.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/BallPlacer.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/FreeKickTaker.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/Passer.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/KeeperPasser.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/active/PassReceiver.cpp
        # passive
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/BallAvoider.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/BallDefender.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/Formation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/roles/passive/RobotDefender.cpp
)
target_include_directories(roboteam_ai_roles
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_roles
        PRIVATE roboteam_utils
        PRIVATE roboteam_networking
        PRIVATE Qt5::Widgets
)
target_compile_options(roboteam_ai_roles PRIVATE "${COMPILER_FLAGS}")

###### PLAYS #######
add_library(roboteam_ai_plays
        ${PROJECT_SOURCE_DIR}/src/stp/Play.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/PlayDecider.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/PlayEvaluator.cpp
        # defensive/
        ${PROJECT_SOURCE_DIR}/src/stp/plays/defensive/DefendPass.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/defensive/DefendShot.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/defensive/KeeperKickBall.cpp
        # offensive/
        ${PROJECT_SOURCE_DIR}/src/stp/plays/offensive/Attack.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/offensive/AttackingPass.cpp
        # referee_specific/
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/AggressiveStopFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/BallPlacementThem.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/BallPlacementUs.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/DefensiveStopFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/FreeKickThem.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/FreeKickUsAtGoal.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/FreeKickUsPass.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/KickOffThem.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/KickOffThemPrepare.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/KickOffUs.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/KickOffUsPrepare.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/PenaltyThem.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/PenaltyThemPrepare.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/PenaltyUs.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/PenaltyUsPrepare.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/plays/referee_specific/FormationPreHalf.cpp
)
target_include_directories(roboteam_ai_plays
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_plays
        PUBLIC roboteam_interface_utils_lib
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE Qt5::Widgets
        PRIVATE SDL2
        PRIVATE NFParam
)
target_compile_options(roboteam_ai_plays PRIVATE "${COMPILER_FLAGS}")

###### EVALUATION #######
add_library(roboteam_ai_evaluation
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/WeHaveBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/TheyHaveBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/TheyDoNotHaveBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallGotShotGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallMovesSlowGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallOnOurSideGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallOnTheirSideGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallInOurDefenseAreaAndStillGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallNotInOurDefenseAreaAndStillGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/FreedomOfRobotsGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallCloseToUsGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallCloseToThemGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/DistanceFromBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/WeHaveMajorityGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/GoalVisionGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/GoalVisionFromBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallIsFreeGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/TheyHaveBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/TheyDoNotHaveBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/NoGoalVisionFromBallGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/global/BallClosestToUsGlobalEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/BallPlacementUsGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/BallPlacementThemGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/HaltGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/KickOffUsGameStateEvaluation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/game_states/KickOffUsOrNormalGameStateEvaluation.cpp
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
        ${PROJECT_SOURCE_DIR}/src/stp/evaluations/position/BlockingEvaluation.cpp
)
target_include_directories(roboteam_ai_evaluation
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_evaluation
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE Qt5::Widgets
        PRIVATE NFParam
)
target_compile_options(roboteam_ai_evaluation PRIVATE "${COMPILER_FLAGS}")

###### COMPUTATION #######
add_library(roboteam_ai_computation
        ${PROJECT_SOURCE_DIR}/src/stp/computations/PositionComputations.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/computations/GoalComputations.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/computations/PassComputations.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/computations/PositionScoring.cpp
)
target_include_directories(roboteam_ai_computation
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_computation
        PRIVATE roboteam_utils
        PRIVATE Qt5::Widgets
        PRIVATE roboteam_networking
        PRIVATE NFParam
)
target_compile_options(roboteam_ai_computation PRIVATE "${COMPILER_FLAGS}")

###### CONTROL #######
add_library(roboteam_ai_control
        ${PROJECT_SOURCE_DIR}/src/control/ControlModule.cpp
        ${PROJECT_SOURCE_DIR}/src/control/ControlUtils.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathPlanning/NumTreesPlanning.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/CollisionDetector.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/PositionControl.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathTracking/DensePathTracking.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathTracking/PidTracking.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathTracking/BBTPathTracking.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/PathPointNode.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathPlanning/PathPlanningAlgorithm.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathTracking/PathTrackingAlgorithm.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/PositionControlUtils.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/WorldObjects.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/BBTrajectory1D.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/BBTrajectory2D.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/Trajectory1D.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/Trajectory2D.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/WorldObjects.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/BBTrajectories/WorldObjects.cpp
        ${PROJECT_SOURCE_DIR}/src/control/AnglePID.cpp
)
target_include_directories(roboteam_ai_control
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_control
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE Qt5::Widgets
)
target_compile_options(roboteam_ai_control PRIVATE "${COMPILER_FLAGS}")

###### INTERFACE *HEADERS* #######
set(INTERFACE_HEADERS
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
        ${PROJECT_SOURCE_DIR}/include/roboteam_ai/interface/widgets/GraphWidget.h
)

###### INTERFACE #######
add_library(roboteam_ai_interface
        ${INTERFACE_HEADERS}
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
        # After removing the interface, only this file should remain
        #${PROJECT_SOURCE_DIR}/src/interface/InterfaceControllerServer.cpp
)
target_include_directories(roboteam_ai_interface
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_interface
        PRIVATE Qt5::Widgets
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE SDL2
        PRIVATE roboteam_interface_utils_lib
)
target_compile_options(roboteam_ai_interface PRIVATE "${COMPILER_FLAGS}")

###### WORLD #######
add_library(roboteam_ai_world
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
target_include_directories(roboteam_ai_world
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_world
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE Qt5::Widgets
)
target_compile_options(roboteam_ai_world PRIVATE "${COMPILER_FLAGS}")

###### MAIN #######
add_executable(roboteam_ai
        ${PROJECT_SOURCE_DIR}/src/STPManager.cpp
        ${PROJECT_SOURCE_DIR}/src/roboteam_ai.cpp
)
target_include_directories(roboteam_ai
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai
        PRIVATE roboteam_utils
        PRIVATE Qt5::Widgets
        PRIVATE roboteam_networking
        PRIVATE SDL2
        # Watch out! The order of these libraries apparently matters!
        PRIVATE roboteam_ai_interface
        PRIVATE roboteam_ai_plays
        PRIVATE roboteam_ai_world
        PRIVATE roboteam_ai_control
        PRIVATE roboteam_ai_roles
        PRIVATE Qt5::Charts
        PRIVATE roboteam_ai_manual
        PRIVATE roboteam_ai_utils
        PRIVATE roboteam_ai_computation
        PRIVATE roboteam_ai_tactics
        PRIVATE roboteam_ai_evaluation
        PRIVATE roboteam_ai_skills
)
target_compile_options(roboteam_ai PRIVATE "${COMPILER_FLAGS}")

###### TESTHELPERS #######
add_library(testHelpers
        ${PROJECT_SOURCE_DIR}/test/helpers/WorldHelper.h
        ${PROJECT_SOURCE_DIR}/test/helpers/WorldHelper.cpp
        ${PROJECT_SOURCE_DIR}/test/helpers/FieldHelper.h
        ${PROJECT_SOURCE_DIR}/test/helpers/FieldHelper.cpp)

target_include_directories(testHelpers
        PRIVATE include/roboteam_ai)

target_link_libraries(testHelpers
        PRIVATE roboteam_utils
        PRIVATE roboteam_networking
        PRIVATE Qt5::Widgets
        PRIVATE roboteam_ai_utils
        )

target_compile_options(testHelpers PRIVATE "${COMPILER_FLAGS}")

###### TEST #######
add_executable(roboteam_ai_tests
        ${PROJECT_SOURCE_DIR}/test/main.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/ControlUtilsTest.cpp
        ${PROJECT_SOURCE_DIR}/test/HelperTests/FieldHelperTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/RefereeTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/BallTests.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/RobotTests.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/FieldComputationTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/HistoryRetrievalTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/WhichRobotHasBallTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/WorldResetTests.cpp
        ${PROJECT_SOURCE_DIR}/test/StpTests/TacticTests.cpp
        ${PROJECT_SOURCE_DIR}/test/StpTests/CorrectPlayTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/BBTrajectory/BBTrajectory1DTest.cpp
        ${PROJECT_SOURCE_DIR}/test/PassingTests/PassingTests.cpp
        )
target_include_directories(roboteam_ai_tests
        PRIVATE include/roboteam_ai
        PRIVATE test
        )
target_link_libraries(roboteam_ai_tests
        PRIVATE GTest::gtest
        PRIVATE GTest::gmock
        PRIVATE testHelpers
        PRIVATE roboteam_utils
        PRIVATE Qt5::Widgets
        PRIVATE roboteam_networking
        PRIVATE SDL2
        # Watch out! The order of these libraries apparently matters!
        PRIVATE roboteam_ai_interface
        PRIVATE roboteam_ai_plays
        PRIVATE roboteam_ai_world
        PRIVATE roboteam_ai_roles
        PRIVATE Qt5::Charts
        PRIVATE roboteam_ai_manual
        PRIVATE roboteam_ai_utils
        PRIVATE roboteam_ai_computation
        PRIVATE roboteam_ai_tactics
        PRIVATE roboteam_ai_evaluation
        PRIVATE roboteam_ai_skills
        PRIVATE roboteam_ai_control
        PRIVATE roboteam_interface_utils_lib
        )

target_compile_options(roboteam_ai_tests PRIVATE "${COMPILER_FLAGS}")

gtest_discover_tests(roboteam_ai_tests)

###### POSITIONSCORINGTEST #######
add_executable(positionScoringTest
        ${PROJECT_SOURCE_DIR}/test/PositionScoringTests/PositionScoringTest.cpp
        )

target_include_directories(positionScoringTest
        PRIVATE test
        PRIVATE include/roboteam_ai
        )

target_link_libraries(positionScoringTest
        PRIVATE Qt5::Widgets
        PRIVATE roboteam_networking
        PRIVATE SDL2
        PRIVATE roboteam_ai_plays
        PRIVATE roboteam_utils
        PRIVATE roboteam_ai_roles
        PRIVATE Qt5::Charts
        PRIVATE roboteam_ai_manual
        PRIVATE roboteam_ai_utils
        PRIVATE roboteam_ai_computation
        PRIVATE roboteam_ai_tactics
        PRIVATE roboteam_ai_evaluation
        PRIVATE roboteam_ai_skills
        PRIVATE testHelpers
        PRIVATE roboteam_ai_world
        PRIVATE roboteam_ai_control
        PRIVATE roboteam_ai_interface
        )