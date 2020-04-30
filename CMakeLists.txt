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
include(ccache)

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

#find_package(Pagmo REQUIRED)
find_package(Qt5Charts REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(SDL2 REQUIRED sdl2)

if (NOT SDL2_FOUND)
    message("SDL2 not found! For linux, run 'sudo apt install libsdl2-dev'")
endif ()

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

set(NEW_SKILLS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Skill.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_skills/SetDribbler.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_skills/Kick.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_skills/Chip.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_skills/Rotate.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_skills/GoToPos.cpp
        )

set(NEW_TACTICS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Tactic.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/TestTactic.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/DriveWithBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/KickAtPos.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/GetBall.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/BlockRobot.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/Receive.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/ChipAtPos.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/Intercept.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_constants/ControlConstants.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/Formation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/GetBallInDirection.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_tactics/BlockBall.cpp
        )

set(NEW_ROLES_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Role.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_roles/TestRole.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_roles/Attacker.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_roles/PassReceiver.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_roles/Passer.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_roles/Defender.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_roles/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_roles/Formation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_roles/Harasser.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_roles/Keeper.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_roles/BallPlacer.cpp
        )

set(NEW_PLAYS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/Play.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/PlayChecker.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/PlayDecider.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_plays/TestPlay.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_plays/Pass.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_plays/Attack.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_plays/Halt.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_plays/Defend.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_plays/DefensiveFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_plays/AggressiveFormation.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/new_plays/BallPlacement.cpp
        )
set(NEW_PLAYS_ANALYSIS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/new_plays_analysis/PassProblem.cpp
        )
set(CONDITIONS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/conditions/HasBall.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/IsInDefenseArea.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/IsRobotClosestToBall.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/CanReflectKick.cpp
        ${PROJECT_SOURCE_DIR}/src/conditions/Condition.cpp
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

set(NEW_PLAYS_ANALYSIS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/new_plays_analysis/PassProblem.cpp
        )

set(INVARIANTS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BaseInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/WeHaveBallInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/HaltGameStateInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallGotShotInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallMovesSlowInvariant.cpp
        ${PROJECT_SOURCE_DIR}/src/stp/invariants/BallOnOurSideInvariant.cpp
        )

set(CONTROL_SOURCES
        ${PROJECT_SOURCE_DIR}/src/control/ControlUtils.cpp

        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathPlanning/NumTreesPlanning.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/CollisionDetector.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/PositionControl.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathPlanning/VoronoiPathPlanning.cpp
        ${PROJECT_SOURCE_DIR}/src/control/positionControl/pathTracking/BasicPathTracking.cpp
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
        ${PROJECT_SOURCE_DIR}/test/World_newTests/HistorySizeTest.cpp
        ${PROJECT_SOURCE_DIR}/test/World_newTests/HistoryRetrievalTest.cpp
        ${PROJECT_SOURCE_DIR}/test/World_newTests/WhichRobotHasBallTest.cpp
        ${PROJECT_SOURCE_DIR}/test/World_newTests/WorldResetTests.cpp
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
        )

set(WORLD_SOURCES
        ${PROJECT_SOURCE_DIR}/src/world/FieldComputations.cpp
        ${PROJECT_SOURCE_DIR}/src/world/Field.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/Ball.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/Robot.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/World.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/WorldData.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/views/WorldDataView.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/views/RobotView.cpp
        ${PROJECT_SOURCE_DIR}/src/world_new/views/BallView.cpp
        )

# Putting it all together so we can use it in both the main executable and the test executable
set(AI_SOURCES
        ${PROJECT_SOURCE_DIR}/src/ApplicationManager.cpp
        ${UTILS_SOURCES}
        ${NEW_SKILLS_SOURCES}
        ${NEW_TACTICS_SOURCES}
        ${NEW_ROLES_SOURCES}
        ${NEW_PLAYS_SOURCES}
        ${INVARIANTS_SOURCES}
        ${CONTROL_SOURCES}
        ${INTERFACE_SOURCES}
        ${WORLD_SOURCES}
        ${MANUAL_SOURCES}
        ${NEW_PLAYS_ANALYSIS_SOURCES}
        )

set(AI_LIBS
        PRIVATE Pagmo::pagmo
        PRIVATE Qt5::Widgets
        PRIVATE Qt5::Gui
        PRIVATE Qt5::Charts
        PRIVATE roboteam_utils
        PRIVATE roboteam_proto
        PRIVATE NFParam
        PRIVATE ${SDL2_LIBRARY}
        )

set(AI_INCLUDES
        INTERFACE include
        PRIVATE include/roboteam_ai
        PRIVATE ${SDL2_INCLUDE_DIRS}
        PRIVATE ${SDL2IMAGE_INCLUDE_DIRS}
        )


# Look for an installation of pagmo in the system.
find_package(Pagmo REQUIRED PATHS /home/jesse/.local/lib/cmake/pagmo)

# Main Executable
add_executable(roboteam_ai src/roboteam_ai.cpp ${AI_SOURCES})
target_link_libraries(roboteam_ai ${AI_LIBS} Pagmo::pagmo)


target_include_directories(roboteam_ai ${AI_INCLUDES})

# Testing #
add_executable(ai_tests ${TEST_SOURCES})
target_link_libraries(ai_tests ${AI_LIBS} ${GTEST_LIB} ai_lib)
target_include_directories(ai_tests ${AI_INCLUDES})
