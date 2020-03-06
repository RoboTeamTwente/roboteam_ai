//
// Created by mrlukasbos on 27-3-19.
//

#include <include/roboteam_ai/world_new/World.hpp>
#include "control/BasicPosControl.h"
#include "control/ControlUtils.h"
#include "interface/api/Input.h"
#include "interface/api/Output.h"
#include "world/FieldComputations.h"

namespace rtt::ai::control {

BasicPosControl::BasicPosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea) : PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {}

/// compare current PID values to those set in the interface
void BasicPosControl::checkInterfacePID() {
    auto newPid = interface::Output::getBasicPid();
    updatePid(newPid);
}

RobotCommand BasicPosControl::getRobotCommand(int robotId, const Vector2 &targetPos) {
        Angle defaultAngle = 0;
        return BasicPosControl::getRobotCommand(robotId, targetPos, defaultAngle);
}
RobotCommand BasicPosControl::getRobotCommand(int robotId, const Vector2 &targetPos, const Angle &targetAngle) {
    auto world = world_new::World::instance()->getWorld().value();
    auto field = world_new::World::instance()->getField().value();
    auto robot = world.getRobotForId(robotId, true).value();
    interface::Input::drawData(interface::Visual::PATHFINDING, {targetPos}, Qt::yellow, robot->getId(), interface::Drawing::CIRCLES, 8, 8, 6);

    Vector2 target = targetPos;
    if (!getCanMoveOutOfField(robot->getId())) {
        if (!FieldComputations::pointIsInField(field, targetPos)) {
            target = ControlUtils::projectPositionToWithinField(field, targetPos);
        }
    }
    if (!getCanMoveInDefenseArea(robot->getId())) {
        if (FieldComputations::pointIsInDefenceArea(field, targetPos)) {
            target = ControlUtils::projectPositionToOutsideDefenseArea(field, targetPos);
        }
    }

    RobotCommand posVelAngle;
    Vector2 error = target - robot->getPos();

    posVelAngle.pos = target;
    posVelAngle.vel = error;
    posVelAngle.angle = (target - robot->getPos()).angle();
    return controlWithPID(robot, posVelAngle);
}
}  // namespace rtt::ai::control
