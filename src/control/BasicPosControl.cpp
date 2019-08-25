//
// Created by mrlukasbos on 27-3-19.
//

#include "../interface/api/Output.h"
#include "../interface/api/Input.h"
#include "BasicPosControl.h"
#include "../world/Robot.h"
#include "../world/Field.h"
#include "ControlUtils.h"

namespace rtt {
namespace ai {
namespace control {

BasicPosControl::BasicPosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
        : PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {

}

RobotCommand BasicPosControl::getRobotCommand(const RobotPtr &robot, const Vector2 &targetPos, const Angle &targetAngle) {

    interface::Input::drawData(interface::Visual::PATHFINDING, {targetPos}, Qt::yellow, robot->id,
            interface::Drawing::CIRCLES, 8, 8, 6);

    Vector2 target = targetPos;
    if (!getCanMoveOutOfField(robot->id)) {
        if (!world::field->pointIsInField(targetPos)) {
            target = ControlUtils::projectPositionToWithinField(targetPos);
        }
    }
    if (!getCanMoveInDefenseArea(robot->id)) {
        if (world::field->pointIsInDefenceArea(targetPos)) {
            target = ControlUtils::projectPositionToOutsideDefenseArea(targetPos);
        }
    }

    RobotCommand posVelAngle;
    Vector2 error = target - robot->pos;

    posVelAngle.pos = target;
    posVelAngle.vel = error;
    posVelAngle.angle = (target - robot->pos).angle();
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
