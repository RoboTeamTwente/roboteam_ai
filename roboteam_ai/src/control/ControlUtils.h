//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_CONTROLUTILS_H
#define ROBOTEAM_AI_CONTROLUTILS_H

#include <roboteam_ai/src/utilities/Hungarian.h>
#include "../world/World.h"
#include "../utilities/Constants.h"
#include "roboteam_utils/Vector2.h"
#include <cmath>
#include "roboteam_utils/Arc.h"

using Vector2 = rtt::Vector2;

namespace rtt {
namespace ai {
namespace control {

class ControlUtils {
    public:
        static double calculateAngularVelocity(double robotAngle, double targetAngle);
        static double TriangleArea(const Vector2 &a, const Vector2 &b, const Vector2 &c);

        static bool pointInTriangle(const Vector2 &pointToCheck,
                const Vector2 &tp1, const Vector2 &tp2, const Vector2 &tp3);

        static bool pointInRectangle(const Vector2 &pointToCheck, const Vector2 &sp1, const Vector2 &sp2,
                const Vector2 &sp3, const Vector2 &sp4);
        static bool pointInRectangle(const Vector2 &pointToCheck, const std::vector<Vector2> &rectangle);

        static double constrainAngle(double angle);
        static double distanceToLine(const Vector2 &PointToCheck, const Vector2 &LineStart, const Vector2 &LineEnd);
        static bool clearLine(const Vector2 &fromPos, const Vector2 &toPos, const world::WorldData &world,
                double safeDistanceFactor, bool keeper = true);
        static double distanceToLineWithEnds(const Vector2 & PointToCheck, const Vector2 &LineStart,
                const Vector2 &LineEnd);
        static double angleDifference(double A1, double A2);
        static int rotateDirection(double currentAngle, double targetAngle);
        static Vector2 projectPositionToWithinField(Vector2 position, float margin = 0.2);
        static Vector2 calculateForce(const rtt::Vector2 &vector, double weight, double minDistance);

        static bool onLineSegment(const Vector2 &p, const Vector2 &q, const Vector2 &r);
        static rtt::Vector2 twoLineIntersection(const Vector2 &a1, const Vector2 &a2, const Vector2 &b1,
                const Vector2 &b2);
        static double twoLineForwardIntersection(const Vector2 &a1, const Vector2 &a2, const Vector2 &b1,
                const Vector2 &b2);
        static int lineOrientation(const Vector2 &p, const Vector2 &q, const Vector2 &r);
        static bool lineSegmentsIntersect(const Vector2 &lineAStart, const Vector2 &lineAEnd, const Vector2 &lineBStart,
                const Vector2 &lineBEnd);
        static rtt::Arc createKeeperArc();
        static Vector2 velocityLimiter(const Vector2 &vel, double maxVel = rtt::ai::Constants::MAX_VEL(),
                double minVel = 0.0);
        static Vector2 accelerationLimiter(Vector2 vel,double maxAcc, double prevVel);
        static double calculateMaxAcceleration(Vector2 vel, double angle);
        static bool robotIsAimedAtPoint(int id, bool ourTeam, const Vector2 &point, double maxDifference = 0.3);
        static bool objectVelocityAimedToPoint(const Vector2 &objectPosition, const Vector2 &velocity,
                const Vector2 &point, double maxDifference = 0.3);
        static std::vector<std::pair<Vector2, Vector2>> calculateClosestPathsFromTwoSetsOfPoints(
                std::vector<Vector2> set1, std::vector<Vector2> set2);
        static double closestEnemyToLineDistance(const Vector2 &fromPos, Vector2 toPos, world::WorldData world,
                                                 bool keeper);
        static bool hasClearVision(int fromID, int towardsID, world::WorldData w, int safeDistanceFactor);
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_CONTROLUTILS_H
