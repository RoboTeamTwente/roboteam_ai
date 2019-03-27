//
// Created by mrlukasbos on 27-3-19.
//

#include "BasicPosControl.h"

namespace rtt {
namespace ai {
namespace control {

PosVelAngle BasicPosControl::getPosVelAngle(RobotPtr robot, Vector2 &targetPos) {

    PosVelAngle posVelAngle;
    Vector2 error;
    error.x = targetPos.x - robot->pos.x;
    error.y = targetPos.y - robot->pos.y;

    if (error.length() < rtt::ai::Constants::ROBOT_RADIUS()) {
        PID.setPID(3.0, 1.0, 0.2);
    } else {
        PID.setPID(3.0, 0.5, 1.5);
    }

    posVelAngle.vel = error;
    return PID(robot, posVelAngle, false);
}

} // control
} // ai
} // rtt
