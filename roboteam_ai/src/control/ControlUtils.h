//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_CONTROLUTILS_H
#define ROBOTEAM_AI_CONTROLUTILS_H

#include <roboteam_ai/src/utilities/Hungarian.h>
#include "../utilities/Constants.h"
#include "roboteam_utils/Vector2.h"
#include "math.h"
#include "../utilities/World.h"
#include "roboteam_utils/Arc.h"

typedef rtt::Vector2 Vector2;

namespace rtt{
namespace ai {
namespace control {

class ControlUtils {
    public:
        static double calculateAngularVelocity(double robotAngle, double targetAngle);
        static double TriangleArea(Vector2 A, Vector2 B, Vector2 C);
        static bool pointInTriangle(Vector2 PointToCheck, Vector2 TP1, Vector2 TP2, Vector2 TP3);
        static bool pointInRectangle(Vector2 PointToCheck, Vector2 SP1, Vector2 SP2, Vector2 SP3, Vector2 SP4);
        static double constrainAngle(double angle);
        static double distanceToLine(Vector2 PointToCheck, Vector2 LineStart, Vector2 LineEnd);
        static bool clearLine(Vector2 fromPos, Vector2 toPos, roboteam_msgs::World world, double safeDistanceFactor, bool keeper = true);
        static double distanceToLineWithEnds(Vector2 PointToCheck, Vector2 LineStart, Vector2 LineEnd);
        static double angleDifference(double A1, double A2);
        static int rotateDirection(double currentAngle, double targetAngle);
        static Vector2 projectPositionToWithinField(Vector2 position, float margin = 0.2);
        static Vector2 calculateForce(rtt::Vector2 vector, double weight, double minDistance);

        static bool hasClearVision(int from, int towards, roboteam_msgs::World world, int safeDistanceFactor);
        static bool onLineSegment(Vector2 p, Vector2 q, Vector2 r);
        static rtt::Vector2 twoLineIntersection(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2);
        static double twoLineForwardIntersection(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2);
        static int lineOrientation(Vector2 p, Vector2 q, Vector2 r);
        static bool lineSegmentsIntersect(Vector2 lineAStart, Vector2 lineAEnd, Vector2 lineBStart, Vector2 lineBEnd);
        static rtt::Arc createKeeperArc();
        static Vector2 VelocityLimiter(Vector2 vel,double maxVel=rtt::ai::Constants::MAX_VEL());
        static Vector2 VelocityLimiter(Vector2 vel,double maxVel, double minVel);
        static bool robotIsAimedAtPoint(int id, bool ourTeam, Vector2 point, double maxDifference = 0.3);

        static std::vector<std::pair<Vector2, Vector2>> calculateClosestPathsFromTwoSetsOfPoints(std::vector<Vector2> set1, std::vector<Vector2> set2);
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_CONTROLUTILS_H
