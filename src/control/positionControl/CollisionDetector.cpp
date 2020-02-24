//
// Created by ratoone on 10-12-19.
//

#include "control/positionControl/CollisionDetector.h"

namespace rtt::ai::control {

bool CollisionDetector::isCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint) {
    bool isFieldColliding = field ? !isPointInsideField(nextPoint) || isPointInDefenseArea(nextPoint) : false;

    return !getRobotCollisionBetweenPoints(initialPoint, nextPoint) && !isFieldColliding;
}

bool CollisionDetector::isPointInsideField(const Vector2& point) { return FieldComputations::pointIsInField(*field, point, Constants::ROBOT_RADIUS()); }

bool CollisionDetector::isPointInDefenseArea(const Vector2& point) {
    return FieldComputations::pointIsInDefenceArea(*field, point, true) || FieldComputations::pointIsInDefenceArea(*field, point, false);
}

std::optional<Vector2> CollisionDetector::getRobotCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint) {
    for (const auto& robot : *robots) {
        // if the initial point is already close to a robot, then either 1. there is a collision, or 2. it is the original robot
        if ((robot->getPos() - initialPoint).length() > Constants::ROBOT_RADIUS() &&
            ControlUtils::distanceToLineWithEnds(robot->getPos(), initialPoint, nextPoint) < this->DEFAULT_ROBOT_COLLISION_RADIUS) {
            return robot->getPos();
        }
    }

    return std::nullopt;
}

std::vector<Vector2> CollisionDetector::getRobotPositions() {
    std::vector<Vector2> robotPositions(robots->size());
    std::transform(robots->begin(), robots->end(), robotPositions.begin(), [](const auto& robot) -> Vector2 { return (robot->getPos()); });
    return robotPositions;
}

void CollisionDetector::setField(const world::Field& field) { this->field = &field; }

void CollisionDetector::setRobotVector(const std::vector<world_new::view::RobotView>& robots) { this->robots = &robots; }

}  // namespace rtt::ai::control