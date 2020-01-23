//
// Created by ratoone on 09-12-19.
//

#ifndef RTT_NUMTREESPLANNING_H
#define RTT_NUMTREESPLANNING_H

#include <list>
#include <roboteam_utils/Vector2.h>
#include <control/numTrees/PathPoint.h>
#include <utilities/GameStateManager.hpp>
#include <utilities/Constants.h>
#include <queue>
#include <control/ControlUtils.h>
#include <interface/api/Output.h>
#include <control/ControlUtils.h>
#include "control/positionControl/CollisionDetector.h"

namespace rtt::ai::control{

class NumTreesPlanning {
private:
    using PathPointer = std::shared_ptr<rtt::ai::control::PathPoint>;

    double DT = 0.1;
    static constexpr double MAX_CALCULATION_TIME = 10.0;
    static constexpr double DEFAULT_ROBOT_COLLISION_RADIUS = 3 * Constants::ROBOT_RADIUS();

    CollisionDetector collisionDetector;

    double
    remainingStraightLinePathLength(const Vector2 &currentPos, const Vector2 &halfwayPos, const Vector2 &finalPos);

    std::list<Vector2>
    backTrackPath(std::shared_ptr<rtt::ai::control::PathPoint> point, const std::shared_ptr<rtt::ai::control::PathPoint> &root);

    std::pair<std::vector<Vector2>, NumTreesPlanning::PathPointer>
    getNewTargets(const NumTreesPlanning::PathPointer &collisionPoint);

    std::list<Vector2> tracePath(const Vector2 &currentPosition, const Vector2 &targetPosition);

    PathPointer computeNewPoint(const Vector2 &targetPosition, const PathPointer &oldPoint, const Vector2 &subTarget);
public:
    explicit NumTreesPlanning(const CollisionDetector& collisionDetector);

    std::list<rtt::Vector2> computePath(const rtt::Vector2 &robotPosition, const rtt::Vector2 &targetPosition);
};

}
#endif //RTT_NUMTREESPLANNING_H
