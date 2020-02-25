//
// Created by ratoone on 10-12-19.
//

#ifndef RTT_COLLISIONDETECTOR_H
#define RTT_COLLISIONDETECTOR_H

#include "control/ControlUtils.h"
#include "world/FieldComputations.h"
#include "world_new/views/RobotView.hpp"

namespace rtt::ai::control {

/**
 * Checks for collision between points and different components of the field: robots, defence area, etc.
 * Check isCollisionBetweenPoints method for more details
 */
class CollisionDetector {
   private:
    const double DEFAULT_ROBOT_COLLISION_RADIUS = 2.0 * Constants::ROBOT_RADIUS();

    std::vector<Vector2> robotPositions;
    const world::Field* field = nullptr;

   public:
    /**
     * Checks if a new point can be followed by a robot from a starting position. This implies having
     * no collisions with other robots, the outside of the field, or the defence area
     * @param initialPoint the starting point
     * @param nextPoint the destination point
     * @return true if there is no collision, false otherwise
     */
    bool isCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint);

    /**
     * Checks whether the line drawn by the two points comes close to any robot (excepting the current one).
     * @param initialPoint
     * @param nextPoint
     * @param currentRobotPosition the current robot position (should be ignored when checking)
     * @return
     */
    std::optional<Vector2> getRobotCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint);

    /**
     * Check if the point is inside the field
     * @param point the point to check
     * @return true if the point is in the field
     */
    bool isPointInsideField(const Vector2& point);

    /**
     * Check if the point is inside the defence area
     * @param point the point to check
     * @return true if the point is in the defence area
     */
    bool isPointInDefenseArea(const Vector2& point);

    std::vector<Vector2> getRobotPositions();

    void setField(const world::Field& field);

    void setRobotPositions(std::vector<Vector2> &robotPositions);
};

}  // namespace rtt::ai::control
#endif  // RTT_COLLISIONDETECTOR_H
