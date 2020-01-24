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

/**
 * Path planning algorithm. See method computePath for details.
 */
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
    /**
     * The collision detector is provided by the position control. This class was intended
     * to be used only with the PositionControl
     * @param collisionDetector
     */
    explicit NumTreesPlanning(const CollisionDetector& collisionDetector);

    /**
     * Computes a path using the implemented algorithm. It takes into account the
     * obstacles present in the field. <br><br>
     * NumTreesPlanning uses an algorithm wrote by an old team member. It tries to
     * trace a path to the destination, and if there is a collision, it traces back
     * and branches into two points to the side of the obstacle, trying them afterwards.
     * @param robotPosition the current robot position
     * @param targetPosition the goal position
     * @return a list of points representing the path
     */
    std::list<Vector2> computePath(const rtt::Vector2 &robotPosition, const rtt::Vector2 &targetPosition);
};

}
#endif //RTT_NUMTREESPLANNING_H
