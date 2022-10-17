//
// Created by ratoone on 10-12-19.
//

#ifndef RTT_COLLISIONDETECTOR_H
#define RTT_COLLISIONDETECTOR_H

#include "utilities/Constants.h"
#include "world/Field.h"
#include "world/views/RobotView.hpp"

namespace rtt::ai::control {

/**
 * Checks for collision between points and different components of the field: robots, defence area, etc.
 * Check isCollisionBetweenPoints method for more details
 */
class CollisionDetector {
   private:
    static constexpr double DEFAULT_ROBOT_COLLISION_RADIUS = 3.0 * Constants::ROBOT_RADIUS();

    std::vector<Vector2> robotPositions;
    const rtt::world::Field* field = nullptr;

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
     * Checks whether the line drawn by the two points comes close to any robot (excepting the current one)
     * and returns that robot's position
     * @param initialPoint
     * @param nextPoint
     * @param currentRobotPosition the current robot position (should be ignored when checking)
     * @return the colliding robot position, or a std::nullopt if there is no collision
     */
    std::optional<Vector2> getRobotCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint);

    /**
     * Check if the point is inside the field
     * @param point the point to check
     * @return true if the point is in the field
     */
    bool isPointInsideField(const Vector2& point);

    /**
     * Check if the line intersects the defense area (adding a margin equal to the robot collision radius)
     * and return the closest point of intersection
     * @param point first point of the line
     * @param nextPoint second point of the line
     * @return the closest intersection with the defense area, or std::nullopt if there is no intersection
     */
    std::optional<Vector2> getDefenseAreaCollision(const Vector2& point, const Vector2& nextPoint);

    /**
     * Calls the defense area collision and robot collision and returns the closest one to the first point
     * @param point first point of the line
     * @param nextPoint second point of the line
     * @return the closest collision point with a robot / the defense area; std::nullopt if no collisions
     */
    std::optional<Vector2> getCollisionBetweenPoints(const Vector2& point, const Vector2& nextPoint);

    void setField(const rtt::world::Field& field);

    void setRobotPositions(std::vector<Vector2>& robotPositions);
};

}  // namespace rtt::ai::control
#endif  // RTT_COLLISIONDETECTOR_H
