//
// Created by mrlukasbos on 27-3-19.
//

#include "../interface/api/Output.h"
#include "../interface/api/Input.h"
#include "BasicPosControl.h"
#include "../world/Robot.h"
#include "ControlUtils.h"

namespace rtt {
namespace ai {
namespace control {

BasicPosControl::BasicPosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
        :PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) { }

RobotCommand BasicPosControl::getRobotCommand(const RobotPtr &robot, const Vector2 &targetPos,
        const Angle &targetAngle) {

    Vector2 target = targetPos;
    if (! getCanMoveOutOfField(robot->id)) {
        target = ControlUtils::projectPositionToWithinField(targetPos, Constants::ROBOT_RADIUS()*1.1);
    }
    if (! getCanMoveInDefenseArea(robot->id)) {
        target = ControlUtils::projectPositionToOutsideDefenseArea(targetPos, Constants::ROBOT_RADIUS()*1.1);
    }

    interface::Input::drawData(interface::Visual::PATHFINDING, {targetPos}, Qt::darkYellow, robot->id,
            interface::Drawing::CIRCLES, 8, 8, 6);
    interface::Input::drawData(interface::Visual::PATHFINDING, {target}, Qt::yellow, robot->id,
            interface::Drawing::CIRCLES, 8, 8, 6);

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
