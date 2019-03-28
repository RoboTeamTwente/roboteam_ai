//
// Created by mrlukasbos on 27-3-19.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include <roboteam_ai/src/utilities/Field.h>
#include "ForcePosControl.h"

namespace rtt {
namespace ai {
namespace control {

PosVelAngle ForcePosControl::getPosVelAngle(RobotPtr robot, Vector2 &targetPos) {
  return calculateForcePosVelAngle(robot, targetPos);
}

Vector2 ForcePosControl::calculateForces(const RobotPtr &robot, const Vector2 &targetPos, double forceRadius) const {
    roboteam_msgs::World world = World::get_world();
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
    if (avoidBall) {
        force += ControlUtils::calculateForce((Vector2) robot->pos - world.ball.pos, FORCE_WEIGHT_BALL, forceRadius);
    }

    // avoid the sides of the field if needed
    if (!canMoveOutOfField) {
        bool pointInField = Field::pointIsInField(robot->pos, POINT_IN_FIELD_MARGIN);

        if (!pointInField) {
            force += ControlUtils::calculateForce(Vector2(-1.0, -1.0) / robot->pos, FORCE_WEIGHT_FIELD_SIDES, 99.9);
        }
    }

    return force;
}

PosVelAngle ForcePosControl::calculateForcePosVelAngle(PosController::RobotPtr robot, Vector2 &targetPos) {
    double forceRadius;
    bool distanceSmallerThanMinForceDistance = (targetPos - robot->pos).length() < Constants::MIN_DISTANCE_FOR_FORCE();
    if (distanceSmallerThanMinForceDistance) {
        forceRadius = Constants::ROBOT_RADIUS_MAX() * 2.0;
        posPID.setPID(3.0, 1.0, 0.2);
    } else {
        forceRadius = Constants::ROBOT_RADIUS_MAX() * 8.0;
        posPID.setPID(3.0, 0.5, 1.5);
    }

    PosVelAngle target;
    auto force = calculateForces(robot, targetPos, forceRadius);
    target.vel = control::ControlUtils::velocityLimiter(force, 3.0);
    return controlWithPID(robot, target);
}


} // control
} // ai
} // rtt

