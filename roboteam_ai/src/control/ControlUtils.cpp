//
// Created by baris on 16/11/18.
//


#include <roboteam_ai/src/utilities/Field.h>
#include "ControlUtils.h"

namespace control {

double ControlUtils::calculateAngularVelocity(double robotAngle, double targetAngle) {
    double direction = 1;               // counter clockwise rotation
    double rotFactor = 8;               // how SLOW the robot rotates when it is near its destination angle

    double angleDiff = targetAngle - robotAngle;
    while (angleDiff < 0) angleDiff += 2*M_PI;
    while (angleDiff > 2*M_PI) angleDiff -= 2*M_PI;
    if (angleDiff > M_PI) {
        angleDiff = 2.0*M_PI - angleDiff;
        direction = - 1;                //  clockwise rotation
    }
    if (angleDiff > 1)angleDiff = 1;
    return direction*(std::pow(rotFactor, angleDiff - 1)*rtt::ai::constants::MAX_ANGULAR_VELOCITY - 1/rotFactor);
}
//Efficient implementation, see this: https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
bool ControlUtils::pointInTriangle(Vector2 PointToCheck, Vector2 TP1, Vector2 TP2, Vector2 TP3) {
    double as_x = PointToCheck.x - TP1.x;
    double as_y = PointToCheck.y - TP1.y;
    bool s_ab = (TP2.x - TP1.x)*as_y - (TP2.y - TP1.y)*as_x > 0;
    if ((((TP3.x - TP1.x)*as_y - (TP3.y - TP1.y)*as_x) > 0) == s_ab) return false;
    return ((((TP3.x - TP2.x)*(PointToCheck.y - TP2.y) - (TP3.y - TP2.y)*(PointToCheck.x - TP2.x)) > 0) == s_ab);
}

double ControlUtils::TriangleArea(Vector2 A, Vector2 B, Vector2 C) {
    return abs((A.x*(B.y - C.y) + B.x*(C.y - A.y) + C.x*(A.y - B.y))*0.5);
}
///Square points must be connected! (e.g. SP1 is connected to SP2 and SP4)
bool ControlUtils::pointInRectangle(Vector2 PointToCheck, Vector2 SP1, Vector2 SP2, Vector2 SP3, Vector2 SP4) {
    if (pointInTriangle(PointToCheck, SP1, SP2, SP3)) {
        return true;
    }
    else return pointInTriangle(PointToCheck, SP4, SP1, SP2);
}
double ControlUtils::constrainAngle(double angle) {
    angle = fmod(angle + M_PI, 2*M_PI);
    if (angle < 0)
        angle += 2*M_PI;
    return angle - M_PI;

}
rtt::Vector2 ControlUtils::getClosestRobot(Vector2 &pos, int &id, bool ourTeam, float &t) {
    auto world = rtt::ai::World::get_world();
    Vector2 closestPos = {420, 420};
    double distance = 99999999;

    for (auto &bot : world.us) {
        if (! (ourTeam && id == static_cast<int>(bot.id))) {
            Vector2 botPos = {bot.pos.x + bot.vel.x*t, bot.pos.y + bot.vel.y*t};
            double deltaPos = (pos - botPos).length();
            if (deltaPos < distance) {
                closestPos = bot.pos;
                distance = deltaPos;
            }

        }

    }
    for (auto &bot : world.them) {
        if (! (! ourTeam && id == static_cast<int>(bot.id))) {
            Vector2 botPos = {bot.pos.x + bot.vel.x*t, bot.pos.y + bot.vel.y*t};
            double deltaPos = (pos - botPos).length();
            if (deltaPos < distance) {
                closestPos = bot.pos;
                distance = deltaPos;
            }
        }
    }
    return closestPos;
}

rtt::Vector2 ControlUtils::getClosestRobot(Vector2 &pos, int &id, bool ourTeam) {
    float t = 0.0f;
    return getClosestRobot(pos, id, ourTeam, t);
}

rtt::Vector2 ControlUtils::getClosestRobot(Vector2 &pos) {
    float t = 0.0f;
    int id = - 1;
    return getClosestRobot(pos, id, true, t);
}

//http://www.randygaul.net/2014/07/23/distance-point-to-line-segment/
double ControlUtils::distanceToLine(Vector2 PointToCheck, Vector2 LineStart, Vector2 LineEnd) {
    Vector2 n = LineEnd - LineStart;
    Vector2 pa = LineStart - PointToCheck;
    Vector2 c = n*(n.dot(pa)/n.dot(n));
    Vector2 d = pa - c;
    return d.length();
}

/// See if a robot has a clear vision towards another robot
bool ControlUtils::hasClearVision(int fromID, int towardsID, roboteam_msgs::World world, int safelyness) {
    double minDistance = rtt::ai::constants::ROBOT_RADIUS*(3*safelyness); // TODO: calibrate Rolf approved
    Vector2 fromPos;
    Vector2 towardsPos;

    for (auto friendly : world.us) {
        if (static_cast<int>(friendly.id) == fromID) {
            fromPos = friendly.pos;
        }
        else if (static_cast<int>(friendly.id) == towardsID) {
            towardsPos = friendly.pos;
        }
    }

    for (auto enemy : world.them) {
        if (distanceToLineWithEnds(enemy.pos, fromPos, towardsPos) < minDistance) {
            return false;
        }
    }

    return true;
}

double ControlUtils::distanceToLineWithEnds(Vector2 pointToCheck, Vector2 lineStart, Vector2 lineEnd) {
    Vector2 n = lineEnd - lineStart;
    Vector2 pa = lineStart - pointToCheck;
    Vector2 c = n*(n.dot(pa)/n.dot(n));
    Vector2 d = pa - c;
    Vector2 A = (pointToCheck - lineStart).project(lineStart, lineEnd);
    Vector2 B = (pointToCheck - lineEnd).project(lineEnd, lineStart);
    if ((A.length() + B.length()) > n.length()) {
        return fmin(pa.length(), (lineEnd - pointToCheck).length());
    }
    else return d.length();
}

//https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool ControlUtils::onLineSegment(Vector2 p, Vector2 q, Vector2 r) {
    return q.x <= fmax(p.x, r.x) && q.x >= fmin(p.x, r.x) &&
            q.y <= fmax(p.y, r.y) && q.y >= fmin(p.y, r.y);

}
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int ControlUtils::lineOrientation(Vector2 p, Vector2 q, Vector2 r) {
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    double val = (q.y - p.y)*(r.x - q.x) -
            (q.x - p.x)*(r.y - q.y);

    if (val == 0) return 0;  // colinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
}
bool ControlUtils::lineSegmentsIntersect(Vector2 lineAStart, Vector2 lineAEnd, Vector2 lineBStart, Vector2 lineBEnd) {
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
    double goalwidth = rtt::ai::Field::get_field().goal_width;
    Vector2 goalPos = rtt::ai::Field::get_our_goal_center();
    double diff = rtt::ai::constants::KEEPER_POST_MARGIN - rtt::ai::constants::KEEPER_CENTREGOAL_MARGIN;

    double radius = diff*0.5 + goalwidth*goalwidth/(8*diff); //Pythagoras' theorem.
    double angle = asin(goalwidth/2/radius); // maximum angle (at which we hit the posts)
    Vector2 center = Vector2(goalPos.x + rtt::ai::constants::KEEPER_CENTREGOAL_MARGIN + radius, 0);
    if (diff > 0) {
        return rtt::Arc(center, radius, M_PI - angle, angle - M_PI);
    }
    else {
        return rtt::Arc(center, radius, angle,
                - angle); //we take the radius from the other side so we have to also flip the arc (yes, confusing)
    }
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
    return abs(angleDif);
}
}//control

