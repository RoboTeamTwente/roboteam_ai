//
// Created by mrlukasbos on 27-3-19.
//

#include <roboteam_ai/src/interface/api/Output.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "BasicPosControl.h"

namespace rtt {
namespace ai {
namespace control {

BasicPosControl::BasicPosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
        : PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {

}

RobotCommand BasicPosControl::getRobotCommand(const RobotPtr &robot, const Vector2 &targetPos, const Angle &targetAngle) {

    interface::Input::drawData(interface::Visual::PATHFINDING, {targetPos}, Qt::yellow, robot->id,
            interface::Drawing::CIRCLES, 8, 8, 6);

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

RobotCommand BasicPosControl::getRobotCommand(const PosController::RobotPtr &robot, const Vector2 &targetPos) {
    Angle defaultAngle = 0;
    return BasicPosControl::getRobotCommand(robot, targetPos, defaultAngle);
}

} // control
} // ai
} // rtt
