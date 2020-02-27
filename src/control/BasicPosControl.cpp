//
// Created by mrlukasbos on 27-3-19.
//

#include "control/BasicPosControl.h"
#include "control/ControlUtils.h"
#include "interface/api/Input.h"
#include "interface/api/Output.h"
#include "world/FieldComputations.h"

namespace rtt::ai::control {

BasicPosControl::BasicPosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea) : PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {}

RobotCommand BasicPosControl::getRobotCommand(world_new::view::WorldDataView *world, const Field *field, const world_new::view::RobotView &robot, const Vector2 &targetPos) {
        Angle defaultAngle = 0;
        return BasicPosControl::getRobotCommand(world, field, robot, targetPos, defaultAngle);
}


RobotCommand BasicPosControl::getRobotCommand(world_new::view::WorldDataView *world, const Field *field, const world_new::view::RobotView &robot, const Vector2 &targetPos, const Angle &targetAngle) {
    interface::Input::drawData(interface::Visual::PATHFINDING, {targetPos}, Qt::yellow, robot->getId(), interface::Drawing::CIRCLES, 8, 8, 6);

    Vector2 target = targetPos;
    if (!getCanMoveOutOfField(robot->getId())) {
        if (!FieldComputations::pointIsInField(*field, targetPos)) {
            target = ControlUtils::projectPositionToWithinField(*field, targetPos);
        }
    }
    if (!getCanMoveInDefenseArea(robot->getId())) {
        if (FieldComputations::pointIsInDefenceArea(*field, targetPos)) {
            target = ControlUtils::projectPositionToOutsideDefenseArea(*field, targetPos);
        }
    }

    RobotCommand posVelAngle;
    Vector2 error = target - robot->getPos();

    posVelAngle.pos = target;
    posVelAngle.vel = error;
    posVelAngle.angle = (target - robot->getPos()).angle();
    return controlWithPID(robot, posVelAngle);
}

/// compare current PID values to those set in the interface
void BasicPosControl::checkInterfacePID() {
    auto newPid = interface::Output::getBasicPid();
    updatePid(newPid);
}

}  // namespace rtt::ai::control
