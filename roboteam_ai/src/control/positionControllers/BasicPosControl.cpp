//
// Created by mrlukasbos on 27-3-19.
//

#include "BasicPosControl.h"

namespace rtt {
namespace ai {
namespace control {

BasicPosControl::BasicPosControl(bool avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
        : PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {

}

PosVelAngle BasicPosControl::getPosVelAngle(const RobotPtr &robot, Vector2 &targetPos) {

    PosVelAngle posVelAngle;
    Vector2 error = targetPos - robot->pos;

    posVelAngle.pos = targetPos;
    posVelAngle.vel = error;
    return controlWithPID(robot, posVelAngle);
}



} // control
} // ai
} // rtt
