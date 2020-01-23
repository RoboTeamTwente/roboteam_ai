//
// Created by ratoone on 10-12-19.
//

#ifndef RTT_COLLISIONDETECTOR_H
#define RTT_COLLISIONDETECTOR_H

#include <world/Field.h>
#include <world_new/Robot.hpp>
#include "control/ControlUtils.h"


namespace rtt::ai::control{

class CollisionDetector {
private:
    const double DEFAULT_ROBOT_COLLISION_RADIUS = 3.0*Constants::ROBOT_RADIUS();

    const std::vector<rtt::world_new::robot::Robot> &robots;
    world::Field* field = nullptr;

public:
    CollisionDetector(const std::vector<rtt::world_new::robot::Robot> &robots);

    bool canFollowPoint(const Vector2& initialPoint, const Vector2& nextPoint);

    /**
     * Checks whether the line drawn by the two points comes close to any robot (excepting the current one).
     * @param initialPoint
     * @param nextPoint
     * @return
     */
    bool isRobotCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint);

    bool isPointInsideField(const Vector2 &point);

    bool isPointInDefenseArea(const Vector2 &point);

    std::vector<const Vector2 *> getRobotPositions();

    void setField(const world::Field &field);
};

}
#endif //RTT_COLLISIONDETECTOR_H
