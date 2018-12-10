//
// Created by baris on 16/11/18.
//


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
    if ((TP3.x - TP1.x)*as_y - (TP3.y - TP1.y)*as_x > 0 == s_ab) return false;
    return ((TP3.x - TP2.x)*(PointToCheck.y - TP2.y) - (TP3.y - TP2.y)*(PointToCheck.x - TP2.x) > 0 == s_ab);
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
rtt::Vector2 ControlUtils::getClosestRobot(rtt::Vector2 &pos, int &id, bool ourTeam, float &t) {
    auto world = rtt::ai::World::get_world();
    rtt::Vector2 closestPos = {420, 420};
    double distance = 99999999;

    for (auto &bot : world.us) {
        if (! (ourTeam && id == bot.id)) {
            rtt::Vector2 botPos = {bot.pos.x + bot.vel.x*t, bot.pos.y + bot.vel.y*t};
            double deltaPos = (pos - botPos).length();
            if (deltaPos < distance) {
                closestPos = bot.pos;
                distance = deltaPos;
            }

        }

    }
    for (auto &bot : world.them) {
        if (! (! ourTeam && id == bot.id)) {
            rtt::Vector2 botPos = {bot.pos.x + bot.vel.x*t, bot.pos.y + bot.vel.y*t};
            double deltaPos = (pos - botPos).length();
            if (deltaPos < distance) {
                closestPos = bot.pos;
                distance = deltaPos;
            }
        }
    }
    return closestPos;
}

rtt::Vector2 ControlUtils::getClosestRobot(rtt::Vector2 &pos, int &id, bool ourTeam) {
    float t = 0.0f;
    return getClosestRobot(pos, id, ourTeam, t);
}

rtt::Vector2 ControlUtils::getClosestRobot(rtt::Vector2 &pos) {
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
    double minDistance = rtt::ai::constants::ROBOT_RADIUS * (3 * safelyness); // TODO: calibrate Rolf approved
    Vector2 fromPos;
    Vector2 towardsPos;

    for (auto friendly : world.us) {
        if (friendly.id == fromID) {
            fromPos = friendly.pos;
        }
        else if (friendly.id == towardsID) {
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
} // control