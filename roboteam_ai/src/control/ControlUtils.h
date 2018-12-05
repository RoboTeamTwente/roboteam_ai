//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_CONTROLUTILS_H
#define ROBOTEAM_AI_CONTROLUTILS_H

#include "../utilities/Constants.h"
#include "roboteam_utils/Vector2.h"
#include "math.h"
#include "../utilities/World.h"


typedef rtt::Vector2 Vector2;
namespace control {
class ControlUtils {
    public:
        static double calculateAngularVelocity(double robotAngle, double targetAngle);
        static double TriangleArea(Vector2 A,Vector2 B,Vector2 C);
        static bool pointInTriangle(Vector2 PointToCheck,Vector2 TP1, Vector2 TP2, Vector2 TP3);
        static bool pointInRectangle(Vector2 PointToCheck,Vector2 SP1, Vector2 SP2, Vector2 SP3,Vector2 SP4);
        static double constrainAngle(double angle);
        static rtt::Vector2 getClosestRobot(rtt::Vector2 &pos, int &id, bool ourTeam, float &t);
        static rtt::Vector2 getClosestRobot(rtt::Vector2 &pos, int &id, bool ourTeam);
        static rtt::Vector2 getClosestRobot(rtt::Vector2 &pos);
        static double distanceToLine(Vector2 PointToCheck, Vector2 LineStart,Vector2 LineEnd);
        static double distanceToLineWithEnds(Vector2 pointToCheck, Vector2 lineStart,Vector2 lineEnd);
        static bool hasClearVision(int from, int towards, roboteam_msgs::World world, int safelyness);
};

}


#endif //ROBOTEAM_AI_CONTROLUTILS_H
