//
// Created by ratoone on 10-12-19.
//

#ifndef RTT_COLLISIONDETECTOR_H
#define RTT_COLLISIONDETECTOR_H

#include <world/Field.h>
#include "world/World.h"
#include "control/ControlUtils.h"
#include "world/Robot.h"


namespace rtt::ai::control{

class CollisionDetector {
private:
    const double DEFAULT_ROBOT_COLLISION_RADIUS = 3.0*Constants::ROBOT_RADIUS();

    world::World& world;
    world::Field& field;
public:
    CollisionDetector(world::World &world, world::Field &field);

    bool canFollowPoint(Vector2 initialPoint, Vector2 nextPoint);

    /**
     * Checks whether the line drawn by the two points comes close to any robot (excepting the current one).
     * @param initialPoint
     * @param nextPoint
     * @return
     */
    bool isRobotCollisionBetweenPoints(Vector2 initialPoint, Vector2 nextPoint);

    std::vector<Vector2 *> getRobotPositions();
};

}
#endif //RTT_COLLISIONDETECTOR_H
