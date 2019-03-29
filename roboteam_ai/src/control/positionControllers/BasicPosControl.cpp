//
// Created by mrlukasbos on 27-3-19.
//

#include "BasicPosControl.h"

namespace rtt {
namespace ai {
namespace control {

PosVelAngle BasicPosControl::getPosVelAngle(RobotPtr robot, Vector2 &targetPos) {

    PosVelAngle posVelAngle;
    Vector2 error = targetPos - robot->pos;

    posVelAngle.pos = error;
    return controlWithPID(robot, posVelAngle);
}

} // control
} // ai
} // rtt
