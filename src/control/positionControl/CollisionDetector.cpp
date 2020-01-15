//
// Created by ratoone on 10-12-19.
//

#include "control/positionControl/CollisionDetector.h"

namespace rtt::ai::control{

CollisionDetector::CollisionDetector(world::World& world, world::Field& field):
world(world), field(field){}

bool CollisionDetector::canFollowPoint(Vector2 initialPoint, Vector2 nextPoint){
    return isRobotCollisionBetweenPoints(initialPoint, nextPoint);
}

bool CollisionDetector::isRobotCollisionBetweenPoints(Vector2 initialPoint, Vector2 nextPoint){
    for (const auto& robot: world.getAllRobots()){
        if (robot->pos != initialPoint && ControlUtils::distanceToLine(robot->pos, initialPoint, nextPoint) < this->DEFAULT_ROBOT_COLLISION_RADIUS){
            return true;
        }
    }

    return false;
}

std::vector<Vector2 *> CollisionDetector::getRobotPositions(){
    auto robots = world.getAllRobots();
    std::vector<Vector2 *> robotPositions(robots.size());
    std::transform(robots.begin(), robots.end(), robotPositions.begin(),
                   [](auto robot) -> Vector2 * { return &(robot->pos); });
    return robotPositions;
}

}