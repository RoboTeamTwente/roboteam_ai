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
    for (const auto& position : robotPositions) {
        // if the initial point is already close to a robot, then either 1. there is a collision, or 2. it is the original robot
        if ((position - initialPoint).length() > Constants::ROBOT_RADIUS() &&
            ControlUtils::distanceToLineWithEnds(position, initialPoint, nextPoint) < this->DEFAULT_ROBOT_COLLISION_RADIUS) {
            return position;
        }
    }

    return std::nullopt;
}

std::vector<Vector2> CollisionDetector::getRobotPositions() {
    return robotPositions;
}

void CollisionDetector::setField(const world::Field& field) { this->field = &field; }

void CollisionDetector::setRobotPositions(std::vector<Vector2> &robotPositions) { this->robotPositions = robotPositions; }

}  // namespace rtt::ai::control