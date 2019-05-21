//
// Created by mrlukasbos on 27-3-19.
//

#include <roboteam_ai/src/interface/api/Output.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "ForcePosControl.h"

namespace rtt {
namespace ai {
namespace control {

ForcePosControl::ForcePosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
        : PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {
}

PosVelAngle ForcePosControl::getPosVelAngle(const RobotPtr &robot, const Vector2 &targetPos, const Angle &targetAngle) {
  return calculateForcePosVelAngle(robot, targetPos);
}

Vector2 ForcePosControl::calculateForces(const RobotPtr &robot, const Vector2 &targetPos, double forceRadius) const {
    auto world = world::world->getWorld();
    Vector2 force = (targetPos - robot->pos);
    force = (force.length() > 3.0) ? force.stretchToLength(3.0) : force;

    // avoid our own robots
    for (auto bot : world.us) {
        force += ControlUtils::calculateForce((Vector2) robot->pos - bot.pos, FORCE_WEIGHT_US, forceRadius);
    }

    // avoid their robots
    for (auto bot : world.them) {
        force += ControlUtils::calculateForce((Vector2) robot->pos - bot.pos, FORCE_WEIGHT_THEM, forceRadius);
    }

    // avoid the ball
    if (getAvoidBallDistance() > 0.0) {
        force += ControlUtils::calculateForce((Vector2) robot->pos - world.ball.pos, FORCE_WEIGHT_BALL, getAvoidBallDistance());
    }

    // avoid the sides of the field if needed
    if (!getCanMoveOutOfField()) {
        bool pointInField = world::field->pointIsInField(robot->pos, POINT_IN_FIELD_MARGIN);

        if (!pointInField) {
            force += ControlUtils::calculateForce(Vector2(-1.0, -1.0) / robot->pos, FORCE_WEIGHT_FIELD_SIDES, 99.9);
        }
    }

    return force;
}

PosVelAngle ForcePosControl::calculateForcePosVelAngle(const PosController::RobotPtr& robot, const Vector2 &targetPos) {
    double forceRadius;
    bool distanceSmallerThanMinForceDistance = (targetPos - robot->pos).length() < Constants::MIN_DISTANCE_FOR_FORCE();
    if (distanceSmallerThanMinForceDistance) {
        forceRadius = Constants::ROBOT_RADIUS_MAX() * 2.0;
    } else {
        forceRadius = Constants::ROBOT_RADIUS_MAX() * 8.0;
    }

    PosVelAngle target;
    auto force = calculateForces(robot, targetPos, forceRadius);
    target.pos = robot->pos + force;
    target.angle = targetPos.toAngle();
    target.vel = {0, 0};
    return controlWithPID(robot, target);
}

void ForcePosControl::checkInterfacePID() {
    auto newPid = interface::Output::getForcePid();
    updatePid(newPid);
}

PosVelAngle ForcePosControl::getPosVelAngle(const PosController::RobotPtr &robot, const Vector2 &targetPos) {
    return PosController::getPosVelAngle(robot, targetPos);
}

} // control
} // ai
} // rtt

