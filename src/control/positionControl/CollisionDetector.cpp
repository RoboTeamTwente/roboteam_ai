//
// Created by martin on 11-5-22.
//

#include "control/positionControl/CollisionDetector.h"

#include <span>

namespace rtt::ai::control {

std::optional<Collision> CollisionDetector::getFirstObjectCollision(PathPoints pathPoints, const int& robotId) {
    for (int i = 0; i < pathPoints.size() && i < TIMELINE_SIZE; i++) {
        auto robotPosition = pathPoints[i];
        auto obstacles = timeline.at(i);

        if (isCollision(robotPosition, obstacles.ball.position, minBallDistance)) {
            return {Collision{obstacles.ball.position, i * TIME_STEP}};
        }

        for (const auto& robot : obstacles.robotsUs) {
            if (robot.robotId == robotId) continue;
            if (isCollision(robotPosition, robot.position, MIN_ROBOT_DISTANCE)) return {Collision{robot.position, i * TIME_STEP}};
        }

        for (const auto& robot : obstacles.robotsThem) {
            if (isCollision(robotPosition, robot.position, MIN_ROBOT_DISTANCE)) return {Collision{robot.position, i * TIME_STEP}};
        }
    }

    return std::nullopt;
}

std::optional<Collision> CollisionDetector::getFirstFieldCollision(PathPoints pathPoints) {
    if (!field.has_value()) return std::nullopt;

    for (int i = 0; i < pathPoints.size(); i++) {
        auto currentPosition = pathPoints[i];
        if (rtt::ai::FieldComputations::pointIsInField(field.value(), currentPosition, rtt::ai::Constants::ROBOT_RADIUS())) continue;

        // Don't care about the field if the robot is already outside the field (i == 0 is the first point of the robot's path, so almost the currentPosition).
        if (i == 0) return std::nullopt;
        return Collision{currentPosition, i * TIME_STEP};
    }

    return std::nullopt;
}

std::optional<Collision> CollisionDetector::getFirstDefenseAreaCollision(PathPoints pathPoints) {
    if (!field.has_value()) return std::nullopt;

    auto ourDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field.value(), true, 0, 0);
    auto theirDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field.value(), false, 0, 0);

    for (size_t i = 0; i < pathPoints.size(); i++) {
        if (!ourDefenseArea.contains(pathPoints[i]) && !theirDefenseArea.contains(pathPoints[i])) continue;

        // Don't care about the defense area if the robot is already in the defense area. It should just get out as fast as possible :)
        if (i == 0) return std::nullopt;  // ???
        return Collision{pathPoints[i], i * TIME_STEP};
    }

    return std::nullopt;
}

CollisionDetector::CollisionDetector() {
    for (int i = 0; i < TIMELINE_SIZE; i++) {
        timeline.emplace_back();
    }
}

void CollisionDetector::updatePositions(const std::vector<rtt::world::view::RobotView>& robots, const rtt::world::view::BallView& ball) {
    for (int i = 0; i < TIMELINE_SIZE; i++) {
        auto& positionsAtTime = timeline.at(i);
        positionsAtTime.robotsThem.clear();
        positionsAtTime.robotsUs.clear();

        double time = i * TIME_STEP;
        for (const auto& robot : robots) {
            auto obstacle = RobotObstacle{robot->getPos() + robot->getVel() * time, robot->getVel(), robot->getId()};
            auto& container = robot->getTeam() == rtt::world::Team::us ? positionsAtTime.robotsUs : positionsAtTime.robotsThem;
            container.emplace_back(obstacle);
        }

        positionsAtTime.ball = BallObstacle{ball->getPos() + ball->getVelocity() * time, ball->getVelocity()};
    }
}

void CollisionDetector::setMinBallDistance(double distance) { minBallDistance = distance; }

std::optional<Collision> CollisionDetector::getFirstCollision(const int& robotId, PathPoints pathPoints) {
    // Collision with objects always takes precedence (i.e. happens BEFORE collision with field or defense area) over field collisions.
    auto collision = getFirstObjectCollision(pathPoints, robotId);
    if (collision.has_value()) return collision;

    collision = getFirstFieldCollision(pathPoints);
    if (collision.has_value()) return collision;

    collision = getFirstDefenseAreaCollision(pathPoints);
    return collision;
}

void CollisionDetector::setField(const std::optional<rtt::world::Field>& newField) { field = newField; }

bool CollisionDetector::isCollision(const Vector2& origin, const Vector2& target, const double& threshold) {
    auto distance = (origin - target).length();
    return distance < threshold;
}

}  // namespace rtt::ai::control
