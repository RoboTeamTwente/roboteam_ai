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

ForcePosControl::ForcePosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
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
    for (auto &ourRobot : world.us) {
        force += ControlUtils::calculateForce(robot->pos - ourRobot.pos, FORCE_WEIGHT_US, forceRadius);
    }

    // avoid their robots
    for (auto &theirRobot : world.them) {
        force += ControlUtils::calculateForce(robot->pos - theirRobot.pos, FORCE_WEIGHT_THEM, forceRadius);
    }

    // avoid the ball
    if (avoidBallDistance > 0.0) {
        force += ControlUtils::calculateForce(robot->pos - world.ball.pos, FORCE_WEIGHT_BALL, forceRadius);
    }

    // avoid the sides of the field if needed
    if (!canMoveOutOfField) {
        bool pointInField = world::field->pointIsInField(robot->pos, POINT_IN_FIELD_MARGIN);

        if (!pointInField) {
            // force += ControlUtils::calculateForce(Vector2(-1.0, -1.0) / robot->pos, FORCE_WEIGHT_FIELD_SIDES, forceRadius);
        }
    }

    if (!canMoveInDefenseArea) {
        auto ourDefenseArea = world::field->getDefenseArea(true, DEFENSE_AREA_MARGIN);
        bool pointInOurDefenseArea = ControlUtils::pointInRectangle(robot->pos, ourDefenseArea);
        if (pointInOurDefenseArea) {
            auto ourGoal = world::field->get_our_goal_center();
            force += ControlUtils::calculateForce(robot->pos - ourGoal, FORCE_WEIGHT_DEFENSE_AREA, 1);
        }
        auto theirDefenseArea = world::field->getDefenseArea(false, DEFENSE_AREA_MARGIN);
        bool pointInTheirDefenseArea = ControlUtils::pointInRectangle(robot->pos, theirDefenseArea);
        if (pointInTheirDefenseArea) {
            auto theirGoal = world::field->get_their_goal_center();
            theirGoal.x = theirGoal.x -0.5;
            std::vector<Vector2> sortedDefenseAreaPoints = theirDefenseArea;
            std::sort(sortedDefenseAreaPoints.begin(), sortedDefenseAreaPoints.end(),
                 [robot](const Vector2 & a, const Vector2 & b) -> bool
                 {
                     return a.dist(robot->pos) < b.dist(robot->pos);
                 });

            double distanceToDefenseArea =
                    control::ControlUtils::distanceToLineWithEnds(robot->pos, sortedDefenseAreaPoints.at(0), sortedDefenseAreaPoints.at(1));



            force += ControlUtils::calculateForce(
                    (robot->pos - theirGoal).stretchToLength(distanceToDefenseArea), FORCE_WEIGHT_DEFENSE_AREA, 1);
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
    target.pos = robot->pos + force;
    target.vel = force;
    return controlWithPID(robot, target);
}

void ForcePosControl::checkInterfacePID() {
    auto newPid = interface::InterfaceValues::getForcePid();
    updatePid(newPid);
}

} // control
} // ai
} // rtt

