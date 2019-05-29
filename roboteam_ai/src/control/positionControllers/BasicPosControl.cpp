//
// Created by mrlukasbos on 27-3-19.
//

#include <roboteam_ai/src/interface/api/Output.h>
#include "BasicPosControl.h"

namespace rtt {
namespace ai {
namespace control {

BasicPosControl::BasicPosControl(bool avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
        : PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {

}

RobotCommand BasicPosControl::getPosVelAngle(const RobotPtr &robot, const Vector2 &targetPos, const Angle &targetAngle) {

    RobotCommand posVelAngle;
    Vector2 error = targetPos - robot->pos;

    posVelAngle.pos = targetPos;
    posVelAngle.vel = error;
    posVelAngle.angle = (targetPos - robot->pos).angle();
    return controlWithPID(robot, posVelAngle);
}

/// compare current PID values to those set in the interface
void BasicPosControl::checkInterfacePID() {
    auto newPid = interface::Output::getBasicPid();
    updatePid(newPid);
}

RobotCommand BasicPosControl::getPosVelAngle(const PosController::RobotPtr &robot, const Vector2 &targetPos) {
    return PosController::getPosVelAngle(robot, targetPos);
}

} // control
} // ai
} // rtt
