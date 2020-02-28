//
// Created by ratoone on 09-12-19.
//

#ifndef RTT_NUMTREESPLANNING_H
#define RTT_NUMTREESPLANNING_H

#include <queue>
#include <vector>
#include "control/numtrees/PathPoint.h"
#include "control/positionControl/CollisionDetector.h"
#include "interface/api/Output.h"
#include "roboteam_utils/Vector2.h"
#include "utilities/Constants.h"
#include "utilities/GameStateManager.hpp"
#include "roboteam_utils/Line.h"
#include "roboteam_utils/Arc.h"

namespace rtt::ai::control {

/**
 * Path planning algorithm. See method computePath for details.
 */
class NumTreesPlanning {
   private:
    using PathPointer = std::shared_ptr<PathPoint>;

    double DT = 0.1;
    static constexpr double MAX_CALCULATION_TIME = 10.0;
    static constexpr double DEFAULT_ROBOT_COLLISION_RADIUS = 4 * Constants::ROBOT_RADIUS();

    CollisionDetector &collisionDetector;

    /// calculate the remaining path length using straight lines from current position to a position halfway and from
    /// halfway to the final position
    double remainingStraightLinePathLength(const Vector2 &currentPos, const Vector2 &halfwayPos, const Vector2 &finalPos);

    /// backTracks the path from endPoint until it hits root and outputs in order from root->endPoint
    std::vector<Vector2> backTrackPath(PathPointer point);

    /// after a collision, get new half-way targets to try to go towards
    std::pair<std::vector<Vector2>, PathPointer> getNewTargets(const PathPointer &collisionPoint);

    /// generates the path between the two points
    std::vector<Vector2> tracePath(const Vector2 &currentPosition, const Vector2 &targetPosition);

    /// create a new pathPoint using a linear acceleration ODE
    PathPointer computeNewPoint(const Vector2 &targetPosition, const PathPointer &oldPoint, const Vector2 &subTarget);

   public:
    /**
     * The collision detector is provided by the position control. This class was intended
     * to be used only with the PositionControl
     * @param collisionDetector
     */
    explicit NumTreesPlanning(CollisionDetector &collisionDetector);

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
    std::vector<Vector2> computePath(const Vector2 &robotPosition, const Vector2 &targetPosition);
};

}  // namespace rtt::ai::control
#endif  // RTT_NUMTREESPLANNING_H
