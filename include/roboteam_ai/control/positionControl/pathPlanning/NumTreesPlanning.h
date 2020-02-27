//
// Created by ratoone on 20-02-20.
//

#ifndef RTT_NUMTREESPLANNING_H
#define RTT_NUMTREESPLANNING_H

#include "utilities/Constants.h"
#include "control/positionControl/CollisionDetector.h"
#include "control/positionControl/PathPointNode.h"
#include <queue>

namespace rtt::ai::control{
class NumTreesPlanning {
private:
    static constexpr double AVOIDANCE_DISTANCE = 4 * Constants::ROBOT_RADIUS();
    static constexpr double TARGET_THRESHOLD = 0.1;
    static constexpr int MAX_BRANCHING = 10;

    CollisionDetector &collisionDetector;

    /**
     * Generate 2 new points to the side of the collisionPosition, such that the points and the parent point form
     * an isosceles triangle, with the collisionPosition being the middle of the base.
     * @param parentPoint starting point
     * @param collisionPosition the point to branch from
     * @return
     */
    std::vector<PathPointNode> branchPath(PathPointNode &parentPoint, const Vector2& collisionPosition) const;

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
}

#endif //RTT_NUMTREESPLANNING_H
