//
// Created by mrlukasbos on 27-3-19.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "ForcePosControl.h"

namespace rtt {
namespace ai {
namespace control {

ForcePosControl::ForcePosControl(bool avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
        : PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {
}

PosVelAngle ForcePosControl::getPosVelAngle(const RobotPtr &robot, Vector2 &targetPos) {
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
    if (avoidBallDistance > 0.0) {
        force += ControlUtils::calculateForce((Vector2) robot->pos - world.ball.pos, FORCE_WEIGHT_BALL, forceRadius);
    }

    // avoid the sides of the field if needed
    if (!canMoveOutOfField) {
        bool pointInField = world::field->pointIsInField(robot->pos, POINT_IN_FIELD_MARGIN);

        if (!pointInField) {
            force += ControlUtils::calculateForce(Vector2(-1.0, -1.0) / robot->pos, FORCE_WEIGHT_FIELD_SIDES, 99.9);
        }
    }

    return force;
}

PosVelAngle ForcePosControl::calculateForcePosVelAngle(const PosController::RobotPtr& robot, Vector2 &targetPos) {
    double forceRadius;
    bool distanceSmallerThanMinForceDistance = (targetPos - robot->pos).length() < Constants::MIN_DISTANCE_FOR_FORCE();
    if (distanceSmallerThanMinForceDistance) {
        forceRadius = Constants::ROBOT_RADIUS_MAX() * 2.0;
    } else {
        forceRadius = Constants::ROBOT_RADIUS_MAX() * 8.0;
    }

    PosVelAngle target;
    auto force = calculateForces(robot, targetPos, forceRadius);
    target.pos = targetPos;
    target.vel = control::ControlUtils::velocityLimiter(force, 3.0);
    return controlWithPID(robot, target);
}



} // control
} // ai
} // rtt

