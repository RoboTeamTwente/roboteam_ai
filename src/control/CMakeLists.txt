add_library(control STATIC
        ControlUtils.cpp
        BasicPosControl.cpp
        PosController.cpp
        numtrees/NumTreePosControl.cpp
        numtrees/PathPoint.cpp
        numtrees/Collision.cpp
        shot-controllers/ShotController.cpp
        ball-handling/BallHandlePosControl.cpp
        ball-handling/DribbleBackwards.cpp
        ball-handling/DribbleForwards.cpp
        ball-handling/RotateAroundBall.cpp
        ball-handling/RotateWithBall.cpp
        )

find_package(Qt5Charts REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)

target_include_directories(control
        PUBLIC ../../include/roboteam_ai
        PUBLIC ../..
        )

target_link_libraries(control
        PUBLIC roboteam_utils
        PRIVATE Qt5::Widgets
        PRIVATE Qt5::Gui
        PRIVATE Qt5::Charts
        )

add_library(rtt::control ALIAS control)