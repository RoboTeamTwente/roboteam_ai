//
// Created by baris on 16/11/18.
//

#include "control/ControlUtils.h"
#include <roboteam_utils/Line.h>
#include <world/FieldComputations.h>
#include <utilities/GameStateManager.hpp>
#include "world_new/World.hpp"

namespace rtt::ai::control {
/// Limits velocity to maximum velocity. it defaults to the max velocity stored in Referee.
Vector2 ControlUtils::velocityLimiter(const Vector2 &vel, double maxVel, double minVel, bool listenToReferee) {
    if (listenToReferee) {
        double refereeMaxVel = rtt::ai::GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel;
        if (refereeMaxVel < maxVel) {
            maxVel = refereeMaxVel;
        }
    }

    if (vel.length() > maxVel) {
        return vel.stretchToLength(maxVel);
    } else if (vel.length() < minVel) {
        return vel.stretchToLength(minVel);
    }
    return vel;
}

/// Limits acceleration
Vector2 ControlUtils::accelerationLimiter(const Vector2 &targetVel, const Vector2 &prevVel, const Angle &targetAngle, double sidewaysAcceleration, double forwardsAcceleration,
                                          double sidewaysDeceleration, double forwardsDeceleration) {
    Vector2 deltaVel = targetVel - prevVel;

    // calculate if the robot is driving forwards or sideways
    Angle robotAngleDifference = targetVel.toAngle() - targetAngle;
    Vector2 robotVectorDifference = robotAngleDifference.toVector2();
    double a = fabs(robotVectorDifference.x);
    auto acceleration = sidewaysAcceleration * (1 - a) + forwardsAcceleration * a;
    auto deceleration = sidewaysDeceleration * (1 - a) + forwardsDeceleration * a;
    // a = 0 -> sideways
    // a = 1 -> forwards

    // calculate if the robot is accelerating or decelerating
    Angle accelerationAngleDifference = deltaVel.toAngle() - targetVel.toAngle();
    double b = fabs(accelerationAngleDifference) * M_1_PI;
    auto finalAcceleration = acceleration * (1 - b) + deceleration * b;
    // b = 0 -> acceleration
    // b = 1 -> deceleration

    if (deltaVel.length() < finalAcceleration) {
        return targetVel;
    }
    return prevVel + deltaVel.stretchToLength(finalAcceleration);
}

/// Calculate the force of a given vector + a certain type.
/// the basic formula is: force = weight/distance^2 * unit vector
Vector2 ControlUtils::calculateForce(const Vector2 &vector, double weight, double minDistance) {
    // if the object is close enough, it's forces should affect. Otherwise don't change anything.
    if (vector.length() < minDistance && vector.length2() > 0) {
        return vector.normalize() * (weight / vector.length2());
    }
    return {0, 0};
}

bool ControlUtils::objectVelocityAimedToPoint(const Vector2 &objectPosition, const Vector2 &velocity, const Vector2 &point, double maxDifference) {
    double exactAngleTowardsPoint = (point - objectPosition).angle();

    // Note: The angles should NOT be constrained here. This is necessary.
    return (velocity.length() > 0 && velocity.angle() > exactAngleTowardsPoint - maxDifference / 2 && velocity.angle() < exactAngleTowardsPoint + maxDifference / 2);
}

bool ControlUtils::robotIsAimedAtPoint(int id, bool ourTeam, const Vector2 &point, const world_new::view::WorldDataView world, double maxDifference) {
    auto robot = world.getRobotForId(id, ourTeam);
    if (robot) {
        Angle exactAngleTowardsPoint = (point - (*robot)->getPos());
        return abs(exactAngleTowardsPoint - (*robot)->getAngle()) < maxDifference;
    }
    return false;
}

bool ControlUtils::clearLine(const Vector2 &fromPos, const Vector2 &toPos, const world_new::view::WorldDataView world, double safeDistanceFactor, bool includeKeeper) {
    double minDistance = Constants::ROBOT_RADIUS() * safeDistanceFactor;
    int keeperID = GameStateManager::getRefereeData().blue().goalkeeper();

    for (auto &enemy : world->getThem()) {
        if (!includeKeeper && enemy->getId() == keeperID) continue;
        if (LineSegment(fromPos, toPos).distanceToLine(enemy->getPos()) < minDistance) {
            return false;
        }
    }

    return true;
}

/// Returns point in field closest to a given point.
/// If the point is already in the field it returns the same as the input.
Vector2 ControlUtils::projectPositionToWithinField(const world::Field &field, Vector2 position, double margin) {
    double hFieldLength = field.getFieldLength() / 2;
    position.x = std::min(position.x, hFieldLength - margin);
    position.x = std::max(position.x, -hFieldLength + margin);

    double hFieldWidth = field.getFieldWidth() / 2;
    position.y = std::min(position.y, hFieldWidth - margin);
    position.y = std::max(position.y, -hFieldWidth + margin);

    return position;
}

Vector2 ControlUtils::projectPositionToOutsideDefenseArea(const world::Field &field, Vector2 position, double margin) {
    if (FieldComputations::pointIsInDefenseArea(field, position, true, margin)) {
        position.x = std::max(position.x, field.getLeftPenaltyX() + margin);
        return position;
    }
    if (FieldComputations::pointIsInDefenseArea(field, position, false, margin)) {
        position.x = std::min(position.x, field.getRightPenaltyX() - margin);
        return position;
    }
    return position;
}

const world_new::view::RobotView ControlUtils::getRobotClosestToLine(std::vector<world_new::view::RobotView> robots, Vector2 const &lineStart, Vector2 const &lineEnd, bool lineWithEnds) {
    int maxDist = INT_MAX;
    auto closestRobot = robots.at(0);
    for (auto const &robot : robots) {
        double dist;
        if (lineWithEnds) {
            dist = Line(lineStart, lineEnd).distanceToLine(robot->getPos());
        } else {
            dist = LineSegment(lineStart, lineEnd).distanceToLine(robot->getPos());
        }
        if (dist > maxDist) {
            dist = maxDist;
            closestRobot = robot;
        }
    }
    return closestRobot;
}

Vector2 ControlUtils::getInterceptPointOnLegalPosition(const world::Field &field, Vector2 position, Line line, bool canMoveInDefenseArea, bool canMoveOutOfField, double defenseAreamargin,
                                                       double) {
    LineSegment shotLine(line.start, line.end + (line.end - line.start));
    Vector2 projectPos = shotLine.project(position);
    Vector2 closestPoint = projectPos;

    bool pointInOurDefenseArea = FieldComputations::pointIsInDefenseArea(field, projectPos, true, defenseAreamargin);
    bool pointInTheirDefenseArea = FieldComputations::pointIsInDefenseArea(field, projectPos, false, defenseAreamargin);

    if (!canMoveInDefenseArea && (pointInOurDefenseArea || pointInTheirDefenseArea)) {
        Polygon defenceAreaUs(FieldComputations::getDefenseArea(field, true, defenseAreamargin, field.getBoundaryWidth()));
        Polygon defenceAreaThem(FieldComputations::getDefenseArea(field, false, defenseAreamargin, field.getBoundaryWidth()));

        std::vector<Vector2> intersects = defenceAreaUs.intersections(shotLine);
        std::vector<Vector2> intersectsThem = defenceAreaThem.intersections(shotLine);

            intersects.insert(intersects.end(), intersectsThem.begin(), intersectsThem.end());
            if (intersects.empty()) {
                return projectPos;
            }
            double closestDist = 9e9;
            for (const auto &point: intersects) {
                if (FieldComputations::pointIsInField(field, point, -defenseAreamargin)) {
                    double dist = point.dist(position);
                    if (dist < closestDist) {
                        closestDist = dist;
                        closestPoint = point;
                    }
                }
            }
        }

        if (!canMoveOutOfField && !FieldComputations::pointIsInField(field, closestPoint, -defenseAreamargin)) {
            closestPoint = projectPositionToWithinField(field, projectPos, defenseAreamargin);
        }

    return closestPoint;
}

}  // namespace rtt::ai::control
