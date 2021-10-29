//
// Created by ratoone on 09-03-20.
//

#ifndef RTT_PATHPLANNINGALGORITHM_H
#define RTT_PATHPLANNINGALGORITHM_H

#include "control/positionControl/CollisionDetector.h"

namespace rtt::ai::control {
/**
 * The base class for the path planning algorithms. All future algorithms should inherit
 * from this
 */
class PathPlanningAlgorithm {
   protected:
    CollisionDetector &collisionDetector;

   public:
    /**
     * The collision detector is provided by the position control. This class was intended
     * to be used only with the PositionControl
     * @param collisionDetector
     */
    explicit PathPlanningAlgorithm(CollisionDetector &collisionDetector);

    /**
     * Algorithm specific path computation. It should take into account the obstacles in the field
     * @param robotPosition the current robot position
     * @param targetPosition the goal position
     * @return a list of points representing the path
     */
    virtual std::vector<Vector2> computePath(const Vector2 &robotPosition, Vector2 &targetPosition) = 0;
};
}  // namespace rtt::ai::control

#endif  // RTT_PATHPLANNINGALGORITHM_H
