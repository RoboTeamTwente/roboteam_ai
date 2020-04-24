//
// Created by baris on 16/11/18.
//

#include "control/ControlUtils.h"
#include <roboteam_utils/Line.h>
#include <world/FieldComputations.h>
#include <utilities/GameStateManager.hpp>
#include "world_new/World.hpp"

namespace rtt::ai::control {
// https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool ControlUtils::onLineSegment(const Vector2 &p, const Vector2 &q, const Vector2 &r) {
    return q.x <= fmax(p.x, r.x) && q.x >= fmin(p.x, r.x) && q.y <= fmax(p.y, r.y) && q.y >= fmin(p.y, r.y);
} // Code clone

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int ControlUtils::lineOrientation(const Vector2 &p, const Vector2 &q, const Vector2 &r) {
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear

    return (val > 0) ? 1 : 2;  // clock or counterclock wise
}
bool ControlUtils::lineSegmentsIntersect(const Vector2 &lineAStart, const Vector2 &lineAEnd, const Vector2 &lineBStart, const Vector2 &lineBEnd) {
    int o1 = lineOrientation(lineAStart, lineAEnd, lineBStart);
    int o2 = lineOrientation(lineAStart, lineAEnd, lineBEnd);
    int o3 = lineOrientation(lineBStart, lineBEnd, lineAStart);
    int o4 = lineOrientation(lineBStart, lineBEnd, lineAEnd);

    // General case
    if (o1 != o2 && o3 != o4) return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onLineSegment(lineAStart, lineBStart, lineAEnd)) return true;

    // p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onLineSegment(lineAStart, lineBEnd, lineAEnd)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onLineSegment(lineBStart, lineAStart, lineBEnd)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    return o4 == 0 && onLineSegment(lineBStart, lineAEnd, lineBEnd);
} // Code clone

// Computes the absolute difference between 2 angles (the shortest orientation direction)
/// both angles must go from[-pi,pi]!!
double ControlUtils::angleDifference(double A1, double A2) {
    double angleDif = A1 - A2;
    if (angleDif < -M_PI) {
        angleDif += 2 * M_PI;
    } else if (angleDif > M_PI) {
        angleDif -= 2 * M_PI;
    }
    return fabs(angleDif);
} // Code clone

// returns the side of rotation that is best from this angle.
int ControlUtils::rotateDirection(double currentAngle, double targetAngle) {
    double angDif = angleDifference(currentAngle, targetAngle);
    double checkForward = Angle(currentAngle + angDif).getAngle();
    double checkBackward = Angle(currentAngle - angDif).getAngle();
    if (abs(checkForward - targetAngle) < abs(checkBackward - targetAngle)) {
        return 1;  // forwards
    } else {
        return -1;  // backwards
    }
}

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

/// Get the intersection of two lines
Vector2 ControlUtils::twoLineIntersection(const Vector2 &a1, const Vector2 &a2, const Vector2 &b1, const Vector2 &b2) {
    // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    double denominator = ((a1.x - a2.x) * (b1.y - b2.y) - (a1.y - a2.y) * (b1.x - b2.x));
    if (denominator != 0) {
        double numerator = ((a1.x - b1.x) * (b1.y - b2.y) - (a1.y - b1.y) * (b1.x - b2.x));
        double t = numerator / denominator;
        return (a1 + (Vector2){t, t} * (a2 - a1));
    } else {
        return Vector2();
    }
} // Code clone

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

}  // namespace rtt::ai::control
