//
// Created by ratoone on 10-12-19.
//

#include "control/positionControl/CollisionDetector.h"

#include "control/ControlUtils.h"
#include "world/FieldComputations.h"

namespace rtt::ai::control {

bool CollisionDetector::isCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint) {
    bool isFieldColliding = field ? !isPointInsideField(nextPoint) || getDefenseAreaCollision(initialPoint, nextPoint) : false;

    // colliding with the outside of the field, the defense area, or collision with a robot
    return isFieldColliding || getRobotCollisionBetweenPoints(initialPoint, nextPoint);
}

std::optional<Vector2> CollisionDetector::getCollisionBetweenPoints(const Vector2& point, const Vector2& nextPoint) {
    auto robotCollision = getRobotCollisionBetweenPoints(point, nextPoint);
    auto defenseCollision = getDefenseAreaCollision(point, nextPoint);

    return (defenseCollision.value_or(nextPoint) - point).length2() < (robotCollision.value_or(nextPoint) - point).length2() ? defenseCollision : robotCollision;
}

bool CollisionDetector::isPointInsideField(const Vector2& point) { return FieldComputations::pointIsInField(*field, point, Constants::ROBOT_RADIUS()); }

std::optional<Vector2> CollisionDetector::getDefenseAreaCollision(const Vector2& point, const Vector2& nextPoint) {
    auto ourDefenseCollision = FieldComputations::lineIntersectionWithDefenseArea(*field, true, point, nextPoint, DEFAULT_ROBOT_COLLISION_RADIUS);
    if (ourDefenseCollision) {
        return *ourDefenseCollision;
    }

    auto theirDefenseCollision = FieldComputations::lineIntersectionWithDefenseArea(*field, false, point, nextPoint, DEFAULT_ROBOT_COLLISION_RADIUS);
    if (!theirDefenseCollision) {
        return std::nullopt;
    }
    return *theirDefenseCollision;
}

std::optional<Vector2> CollisionDetector::getRobotCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint) {
    for (const auto& position : robotPositions) {
        // if the initial point is already close to a robot, then either 1. there is a collision, or 2. it is the original robot
        if ((position - initialPoint).length() > Constants::ROBOT_RADIUS() && LineSegment(initialPoint, nextPoint).distanceToLine(position) < DEFAULT_ROBOT_COLLISION_RADIUS) {
            return position;
        }
    }

    return std::nullopt;
}

void CollisionDetector::setField(const rtt::world::Field& field_) { this->field = &field_; }

void CollisionDetector::setRobotPositions(std::vector<Vector2>& robotPositions_) { this->robotPositions = robotPositions_; }

}  // namespace rtt::ai::control