cmake_minimum_required(VERSION 3.4)
project(roboteam_ai)


#SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -O0 -Wall --coverage")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++11 ${CMAKE_CXX_FLAGS} -g3 -O0 -Wall --coverage")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 ${CMAKE_CXX_FLAGS} -Werror -O1")

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(cotire)

SET(Qt5Widgets_DIR "/usr/include/x86_64-linux-gnu/qt5/QtWidgets")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC OFF)

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

set(JSON_SOURCES)

set(BT_SOURCES)

set(TACTIC_SOURCES)

set(IO_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/IOManager.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/io/IOManager.cpp)

set(UTILS_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Constants.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/utilities/Constants.cpp
        )


set(SKILLS_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/BallPlacementWithInterface.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/skills/BallPlacementWithInterface.h
        )

set(CONDITIONS_SOURCES)

set(CONTROL_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ControlUtils.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ControlUtils.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/pid.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/pid.h

        # PositionControllers
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/RobotCommand.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/BasicPosControl.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/BasicPosControl.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/PosController.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/PosController.h

        # NumTrees
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/numTrees/NumTreePosControl.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/numTrees/NumTreePosControl.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/numTrees/PathPoint.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/numTrees/PathPoint.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/numTrees/Collision.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/numTrees/Collision.h

        # BallHandling
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ballHandling/BallHandlePosControl.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ballHandling/BallHandlePosControl.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ballHandling/DribbleBackwards.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ballHandling/DribbleBackwards.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ballHandling/DribbleForwards.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ballHandling/DribbleForwards.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ballHandling/RotateAroundBall.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ballHandling/RotateAroundBall.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ballHandling/RotateWithBall.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/control/ballHandling/RotateWithBall.h
        )

set(TEST_SOURCES)

set(INTERFACE_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/mainWindow.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/mainWindow.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/widget.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/widget.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/api/Input.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/api/Input.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/api/Output.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/api/Output.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/RobotsWidget.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/RobotsWidget.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/PidBox.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/PidBox.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/PidsWidget.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/PidsWidget.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/MainControlsWidget.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/MainControlsWidget.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/VisualizationSettingsWidget.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/VisualizationSettingsWidget.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/CheckboxWidget.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/widgets/CheckboxWidget.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/api/Toggles.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/interface/api/Toggles.cpp
        )

set(WORLD_SOURCES
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/WorldData.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/Robot.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/Robot.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/Ball.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/Ball.h

        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/World.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/World.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/Field.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/Field.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/WorldManager.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/WorldManager.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/History.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/History.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/FutureWorld.cpp
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/FutureWorld.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/Team.h
        ${PROJECT_SOURCE_DIR}/roboteam_ai/src/world/WhichRobots.h)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Main Executable
add_executable(Roboteam_AI
        roboteam_ai/src/roboteam_ai.cpp
        roboteam_ai/src/ApplicationManager.cpp
        roboteam_ai/src/ApplicationManager.h
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
        ${WORLD_SOURCES})

add_dependencies(Roboteam_AI ${catkin_EXPORTED_TARGETS})
target_link_libraries(Roboteam_AI ${catkin_LIBRARIES} Qt5::Widgets Qt5::Gui)
cotire(Roboteam_AI)

#############
## Testing ##
#############
catkin_add_gtest(ai_tests
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

target_compile_options(ai_tests PUBLIC --coverage)
target_link_libraries(ai_tests
        ${catkin_LIBRARIES}
        gcov
        Qt5::Widgets Qt5::Gui)
cotire(ai_tests)

# this creates a directory for the jsons where  ROS launch scripts can access it
# then the json folder gets linked to that directory, so also ROS launch scripts can profit from refreshing JSONS on runtime
ADD_CUSTOM_TARGET(link_target ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/roboteam_ai/src
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/roboteam_ai/src/jsons ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/roboteam_ai/src/jsons
        DEPENDS Roboteam_AI ai_tests
        )

include(CodeCoverage)
APPEND_COVERAGE_COMPILER_FLAGS()
SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${PROJECT_NAME}_coverage EXECUTABLE ai_tests)
