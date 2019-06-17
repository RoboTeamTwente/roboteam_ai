//
// Created by baris on 16/11/18.
//


#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/utilities/GameStateManager.hpp>
#include <roboteam_utils/Line.h>
#include "ControlUtils.h"
#include "../world/World.h"
#include "../world/WorldData.h"

namespace rtt {
namespace ai {
namespace control {


// Efficient implementation, see this: https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
/// Returns if PointToCheck is within the triangle constructed by three points.
bool ControlUtils::pointInTriangle(const Vector2 &pointToCheck,
        const Vector2 &tp1, const Vector2 &tp2, const Vector2 &tp3) {

    double as_x = pointToCheck.x - tp1.x;
    double as_y = pointToCheck.y - tp1.y;
    bool s_ab = (tp2.x - tp1.x)*as_y - (tp2.y - tp1.y)*as_x > 0;
    if ((((tp3.x - tp1.x)*as_y - (tp3.y - tp1.y)*as_x) > 0) == s_ab) return false;
    return ((((tp3.x - tp2.x)*(pointToCheck.y - tp2.y) - (tp3.y - tp2.y)*(pointToCheck.x - tp2.x)) > 0) == s_ab);
}

/// Returns the area of a triangle constructed from three points.
double ControlUtils::TriangleArea(const Vector2 &a, const Vector2 &b, const Vector2 &c) {
    return abs((a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y))*0.5);
}

///Square points must be connected! (e.g. SP1 is connected to SP2 and SP4)
bool ControlUtils::pointInRectangle(const Vector2 &pointToCheck, const Vector2 &sp1, const Vector2 &sp2,
        const Vector2 &sp3, const Vector2 &sp4) {

    if (pointInTriangle(pointToCheck, sp1, sp2, sp3)) {
        return true;
    }
    else return pointInTriangle(pointToCheck, sp4, sp1, sp3);
}

bool ControlUtils::pointInRectangle(const Vector2 &pointToCheck, const std::vector<Vector2> &rectangle) {
    if (rectangle.size() == 4) {
        return pointInRectangle(pointToCheck, rectangle[0], rectangle[1], rectangle[2], rectangle[3]);
    }
    return false;
}

/// Maps the input angle to be within the range of 0 - 2PI
double ControlUtils::constrainAngle(double angle) {
    angle = fmod(angle + M_PI, 2*M_PI);
    if (angle < 0)
        angle += 2*M_PI;
    return angle - M_PI;
}

bool ControlUtils::isPointProjectedOnLineSegment(const Vector2 &pointToCheck, const Vector2 &lineBegin,
                                                 const Vector2 &lineEnd) {

    Vector2 projectionPoint = pointToCheck.project(lineBegin, lineEnd);
    double xMin = std::min(lineBegin.x, lineEnd.x);
    double xMax = std::max(lineBegin.x, lineEnd.x);
    double yMin = std::min(lineBegin.y, lineEnd.y);
    double yMax = std::max(lineBegin.y, lineEnd.y);

    return (projectionPoint.x > xMin && projectionPoint.x < xMax && projectionPoint.y > yMin && projectionPoint.y < yMax);
}

/// Get the distance from PointToCheck towards a line - the line is infinitely long
//http://www.randygaul.net/2014/07/23/distance-point-to-line-segment/
double ControlUtils::distanceToLine(const Vector2 &PointToCheck, const Vector2 &LineStart, const Vector2 &LineEnd) {
    Vector2 n = LineEnd - LineStart;
    Vector2 pa = LineStart - PointToCheck;
    Vector2 c = n*(n.dot(pa)/n.dot(n));
    Vector2 d = pa - c;
    return d.length();
}

bool ControlUtils::clearLine(const Vector2 &fromPos, const Vector2 &toPos,
        const world::WorldData &world, double safeDistanceFactor, bool includeKeeper) {

    double minDistance = Constants::ROBOT_RADIUS() * safeDistanceFactor;
    int keeperID = GameStateManager::getRefereeData().them.goalie;

    for (auto &enemy : world.them) {
        if(!includeKeeper && enemy->id == keeperID) continue;
        if (distanceToLineWithEnds(enemy->pos, fromPos, toPos) < minDistance) {
            return false;
        }
    }

    return true;
}

/// Get the distance from PointToCheck towards a line, the line is not infinite.
double ControlUtils::distanceToLineWithEnds(const Vector2 &pointToCheck,
        const Vector2 &lineStart, const Vector2 &lineEnd) {

    Vector2 line = lineEnd - lineStart;
    Vector2 diff = pointToCheck - lineStart;
    double dot = line.x*diff.x + line.y*diff.y;
    double len_sq = line.y*line.y + line.x*line.x;
    double param = - 1;
    if (len_sq != 0) {
        param = dot/len_sq;
    }
    if (param < 0) {
        param = 0;
    }
    else if (param > 1) {
        param = 1;
    }
    Vector2 project = lineStart + line*param;
    Vector2 distDiff = pointToCheck - project;
    return distDiff.length();
}

// https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool ControlUtils::onLineSegment(const Vector2 &p, const Vector2 &q, const Vector2 &r) {
    return q.x <= fmax(p.x, r.x) && q.x >= fmin(p.x, r.x) &&
            q.y <= fmax(p.y, r.y) && q.y >= fmin(p.y, r.y);

}
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int ControlUtils::lineOrientation(const Vector2 &p, const Vector2 &q, const Vector2 &r) {
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    double val = (q.y - p.y)*(r.x - q.x) -
            (q.x - p.x)*(r.y - q.y);

    if (val == 0) return 0;  // colinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
}
bool ControlUtils::lineSegmentsIntersect(const Vector2 &lineAStart, const Vector2 &lineAEnd,
        const Vector2 &lineBStart, const Vector2 &lineBEnd) {

    int o1 = lineOrientation(lineAStart, lineAEnd, lineBStart);
    int o2 = lineOrientation(lineAStart, lineAEnd, lineBEnd);
    int o3 = lineOrientation(lineBStart, lineBEnd, lineAStart);
    int o4 = lineOrientation(lineBStart, lineBEnd, lineAEnd);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onLineSegment(lineAStart, lineBStart, lineAEnd)) return true;

    // p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onLineSegment(lineAStart, lineBEnd, lineAEnd)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onLineSegment(lineBStart, lineAStart, lineBEnd)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onLineSegment(lineBStart, lineAEnd, lineBEnd)) return true;

    return false; // Doesn't fall in any of the above cases

}
rtt::Arc ControlUtils::createKeeperArc() {
    double goalwidth = rtt::ai::world::field->get_field().goal_width;
    Vector2 goalPos = rtt::ai::world::field->get_our_goal_center();
    double diff = rtt::ai::Constants::KEEPER_POST_MARGIN() - rtt::ai::Constants::KEEPER_CENTREGOAL_MARGIN();

    double radius = diff*0.5 + goalwidth*goalwidth/(8*diff); //Pythagoras' theorem.
    double angle = asin(goalwidth/2/radius); // maximum angle (at which we hit the posts)
    Vector2 center = Vector2(goalPos.x + rtt::ai::Constants::KEEPER_CENTREGOAL_MARGIN() + radius, 0);
    return diff > 0 ? rtt::Arc(center, radius, M_PI - angle, angle - M_PI) :
           rtt::Arc(center, radius, angle, - angle);
}

//Computes the absolute difference between 2 angles (the shortest orientation direction)
///both angles must go from[-pi,pi]!!
double ControlUtils::angleDifference(double A1, double A2) {
    double angleDif = A1 - A2;
    if (angleDif < - M_PI) {
        angleDif += 2*M_PI;
    }
    else if (angleDif > M_PI) {
        angleDif -= 2*M_PI;
    }
    return fabs(angleDif);
}

//returns the side of rotation that is best from this angle.
int ControlUtils::rotateDirection(double currentAngle, double targetAngle) {
    double angDif = angleDifference(currentAngle, targetAngle);
    double checkForward = constrainAngle(currentAngle + angDif);
    double checkBackward = constrainAngle(currentAngle - angDif);
    if (abs(checkForward - targetAngle) < abs(checkBackward - targetAngle)) {
        return 1;       //forwards
    }
    else return - 1;    //backwards
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
    }
    else if (vel.length() < minVel) {
        return vel.stretchToLength(minVel);
    }
    return vel;
}


/// Limits acceleration
Vector2 ControlUtils::accelerationLimiter(const Vector2 &targetVel, const Vector2 &prevVel, const Angle &targetAngle,
        double sidewaysAcceleration, double forwardsAcceleration,
        double sidewaysDeceleration, double forwardsDeceleration) {

    Vector2 deltaVel = targetVel - prevVel;

    // calculate if the robot is driving forwards or sideways
    Angle robotAngleDifference = targetVel.toAngle() - targetAngle;
    Vector2 robotVectorDifference = robotAngleDifference.toVector2();
    double a = fabs(robotVectorDifference.x);
    auto acceleration = sidewaysAcceleration * (1-a) + forwardsAcceleration * a;
    auto deceleration = sidewaysDeceleration * (1-a) + forwardsDeceleration * a;
    // a = 0 -> sideways
    // a = 1 -> forwards

    // calculate if the robot is accelerating or decelerating
    Angle accelerationAngleDifference = deltaVel.toAngle() - targetVel.toAngle();
    double b = fabs(accelerationAngleDifference) * M_1_PI;
    auto finalAcceleration = acceleration * (1-b) + deceleration * b;
    // b = 0 -> acceleration
    // b = 1 -> deceleration

    if (deltaVel.length() < finalAcceleration) {
        return targetVel;
    }
    return prevVel + deltaVel.stretchToLength(finalAcceleration);
}

/// Get the intersection of two lines
Vector2 ControlUtils::twoLineIntersection(const Vector2 &a1, const Vector2 &a2, const Vector2 &b1, const Vector2 &b2) {
    //https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    double denominator = ((a1.x - a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x - b2.x));
    if (denominator != 0) {
        double numerator = ((a1.x - b1.x)*(b1.y - b2.y) - (a1.y - b1.y)*(b1.x - b2.x));
        double t = numerator/denominator;
        return (a1 + (Vector2) {t, t}*(a2 - a1));
    }
    else
        return Vector2();
}
/// returns true if the line intersects in the positive extension from point a1 to a2 with the extended line through b1 and b2
double ControlUtils::twoLineForwardIntersection(const Vector2& a1,const Vector2& a2,const Vector2& b1,const Vector2& b2) {
    double denominator = ( (a1.x - a2.x)*(b1.y - b2.y) - (a1.y - a2.y)*(b1.x - b2.x) );
    if (denominator != 0) {
        double numerator = ( (a1.x - b1.x)*(b1.y - b2.y) - (a1.y - b1.y)*(b1.x - b2.x) );
        double t =  numerator / denominator;
        return t;
    }
    else
        return -1.0;
}
/// Returns point in field closest to a given point.
/// If the point is already in the field it returns the same as the input.
Vector2 ControlUtils::projectPositionToWithinField(Vector2 position, double margin) {
    auto field = world::field->get_field();

    double hFieldLength = field.field_length/2;
    position.x = std::min(position.x, hFieldLength - margin);
    position.x = std::max(position.x, - hFieldLength + margin);

    double hFieldWidth = field.field_width/2;
    position.y = std::min(position.y, hFieldWidth - margin);
    position.y = std::max(position.y, - hFieldWidth + margin);

    return position;
}

Vector2 ControlUtils::projectPositionToOutsideDefenseArea(Vector2 position, double margin) {
    if (world::field->pointIsInDefenceArea(position, true, margin)) {
        position.x = std::max(position.x, world::field->get_field().left_penalty_line.begin.x + margin);
        return position;
    }
    if (world::field->pointIsInDefenceArea(position, false, margin)) {
        position.x = std::min(position.x, world::field->get_field().right_penalty_line.begin.x - margin);
        return position;
    }
    return position;
}

/// Calculate the force of a given vector + a certain type.
/// the basic formula is: force = weight/distance^2 * unit vector
Vector2 ControlUtils::calculateForce(const Vector2 &vector, double weight, double minDistance) {

    // if the object is close enough, it's forces should affect. Otherwise don't change anything.
    if (vector.length() < minDistance && vector.length2() > 0) {
        return vector.normalize()*(weight/vector.length2());
    }
    return {0, 0};
}

bool ControlUtils::robotIsAimedAtPoint(int id, bool ourTeam, const Vector2 &point, double maxDifference) {
    auto robot = world::world->getRobotForId(id, ourTeam);
    if (robot) {
        Angle exactAngleTowardsPoint = (point - robot->pos);

        return abs(exactAngleTowardsPoint - robot->angle) < maxDifference;
    }
    return false;
}

bool ControlUtils::objectVelocityAimedToPoint(const Vector2 &objectPosition, const Vector2 &velocity,
        const Vector2 &point, double maxDifference) {

    double exactAngleTowardsPoint = (point - objectPosition).angle();

    // Note: The angles should NOT be constrained here. This is necessary.
    return (velocity.length() > 0
    && velocity.angle() > exactAngleTowardsPoint - maxDifference/2
    && velocity.angle() < exactAngleTowardsPoint + maxDifference/2);

}


const world::World::RobotPtr ControlUtils::getRobotClosestToLine(std::vector<world::World::RobotPtr> robots, Vector2 const &lineStart, Vector2 const &lineEnd, bool lineWithEnds) {
    int maxDist = INT_MAX;
    auto closestRobot = robots.at(0);
    for (auto const & robot : robots) {
        double dist;
        if (lineWithEnds) {
            dist = distanceToLine(robot->pos, lineStart, lineEnd);
        } else {
            dist = distanceToLineWithEnds(robot->pos, lineStart, lineEnd);
        }
        if (dist > maxDist) {
            dist = maxDist;
             closestRobot = robot;
        }
    }

    return closestRobot;
}

    Vector2 ControlUtils::getInterceptPointOnLegalPosition(Vector2 position, Line line, bool canMoveInDefenseArea, bool canMoveOutOfField, double defenseAreamargin, double outOfFieldMargin) {
        LineSegment shotLine(line.start, line.end + (line.end - line.start).stretchToLength(100.0));
        Vector2 projectPos = shotLine.project(position);
        Vector2 closestPoint = projectPos;

        bool pointInOurDefenseArea = world::field->pointIsInDefenceArea(projectPos, true, defenseAreamargin);
        bool pointInTheirDefenseArea = world::field->pointIsInDefenceArea(projectPos, false, defenseAreamargin);

        if (!canMoveInDefenseArea && (pointInOurDefenseArea || pointInTheirDefenseArea)) {

            Polygon defenceAreaUs(world::field->getDefenseArea(true, defenseAreamargin, true));
            Polygon defenceAreaThem(world::field->getDefenseArea(false, defenseAreamargin, true));


            std::vector<Vector2> intersects = defenceAreaUs.intersections(shotLine);
            std::vector<Vector2> intersectsThem = defenceAreaThem.intersections(shotLine);

            intersects.insert(intersects.end(), intersectsThem.begin(), intersectsThem.end());
            if (intersects.empty()) {
                return projectPos;
            }
            double closestDist = DBL_MAX;
            for (const auto &point :intersects) {
                if (world::field->pointIsInField(point, defenseAreamargin)) {
                    double dist = point.dist(position);
                    if (dist < closestDist) {
                        closestDist = dist;
                        closestPoint = point;
                    }
                }
            }
        }

        if (!canMoveOutOfField && !world::field->pointIsInField(closestPoint, defenseAreamargin)) {
            closestPoint = projectPositionToWithinField(projectPos, defenseAreamargin);
        }

        return closestPoint;

    }


} // control
} // ai
} // rtt

