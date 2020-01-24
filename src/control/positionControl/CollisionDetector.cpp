//
// Created by ratoone on 10-12-19.
//

#include "control/positionControl/CollisionDetector.h"

namespace rtt::ai::control{

CollisionDetector::CollisionDetector(const std::vector<rtt::world_new::robot::Robot> &robots): robots(robots){}

bool CollisionDetector::canFollowPoint(const Vector2& initialPoint, const Vector2& nextPoint){
    return isRobotCollisionBetweenPoints(initialPoint, nextPoint) ||
            isPointInsideField(nextPoint) ||
            !isPointInDefenseArea(nextPoint);
}

bool CollisionDetector::isPointInsideField(const Vector2 &point){
    return field->pointIsInField(point, Constants::ROBOT_RADIUS());
}

bool CollisionDetector::isPointInDefenseArea(const Vector2 &point){
    return field->pointIsInDefenceArea(point, true) || field->pointIsInDefenceArea(point, false);
}

bool CollisionDetector::isRobotCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint){
    for (const auto& robot: robots){
        if (robot.getPos() != initialPoint && ControlUtils::distanceToLine(robot.getPos(), initialPoint, nextPoint) < this->DEFAULT_ROBOT_COLLISION_RADIUS){
            return true;
        }
    }

    return false;
}

std::vector<const Vector2 *> CollisionDetector::getRobotPositions(){
    std::vector<const Vector2 *> robotPositions(robots.size());
    std::transform(robots.begin(), robots.end(), robotPositions.begin(),
                   [](const auto& robot) -> const Vector2 * { return &(robot.getPos()); });
    return robotPositions;
}

void CollisionDetector::setField(world::Field *field) {
    this->field = field;
}

}