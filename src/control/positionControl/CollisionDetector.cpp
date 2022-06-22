//
// Created by martin on 11-5-22.
//

#include "control/positionControl/CollisionDetector.h"

#include <Tracy.hpp>
#include <span>

#include "control/positionControl/PositionControlUtils.h"

namespace rtt::ai::control {

template <PathPointType T>
std::optional<Collision> CollisionDetector::getFirstObjectCollision(std::span<T> path, int robotId, bool shouldAvoidBall, int timeOffset, int timeLimit) const {
    ZoneScopedN("Get Object Collision");
    for (int i = timeOffset; i < path.size() && i < STEP_COUNT && i <= timeLimit; i++) {
        auto pathPoint = path[i];
        auto obstacles = timeline[i];

        if (shouldAvoidBall && isCollision(pathPoint, obstacles.ball.position, minBallDistance)) {
            return {Collision{obstacles.ball.position, obstacles.ball.velocity, i}};
        }

        for (const auto& [otherRobotId, otherRobot] : obstacles.robotsUs) {
            if (otherRobotId == robotId) continue;
            if (!isCollision(pathPoint, otherRobot.position, MIN_ROBOT_DISTANCE)) continue;
            return {Collision{otherRobot.position, otherRobot.velocity, i}};
        }

        for (const auto& robot : obstacles.robotsThem) {
            if (!isCollision(pathPoint, robot.position, MIN_ROBOT_DISTANCE)) continue;
            return {Collision{robot.position, robot.velocity, i}};
        }
    }

    return std::nullopt;
}

template <PathPointType T>
std::optional<Collision> CollisionDetector::getFirstFieldCollision(std::span<T> path) const {
    ZoneScopedN("Get Field Collision");
    if (!field.has_value()) return std::nullopt;

    for (int i = 0; i < path.size(); i++) {
        auto pathPoint = unwrapPosition(path[i]);
        if (rtt::ai::FieldComputations::pointIsInField(field.value(), pathPoint, rtt::ai::Constants::ROBOT_RADIUS())) continue;

        // Don't care about the field if the robot is already outside the field (i == 0 is the first point of the robot's path, so almost the currentPosition).
        if (i == 0) return std::nullopt;
        return Collision{pathPoint, Vector2{0, 0}, i};
    }

    return std::nullopt;
}

template <PathPointType T>
std::optional<Collision> CollisionDetector::getFirstDefenseAreaCollision(std::span<T> path) const {
    ZoneScopedN("Get Defense Area Collision");
    if (!field.has_value()) return std::nullopt;

    auto ourDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field.value(), true, 0, 0);
    auto theirDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field.value(), false, 0, 0);
    for (int i = 0; i < path.size(); i++) {
        auto pathPoint = unwrapPosition(path[i]);
        if (!ourDefenseArea.contains(pathPoint) && !theirDefenseArea.contains(pathPoint)) continue;

        // Don't care about the defense area if the robot is already in the defense area. It should just get out as fast as possible :)
        if (i == 0) return std::nullopt;  // ???
        return Collision{pathPoint, Vector2{0, 0}, i};
    }

    return std::nullopt;
}

CollisionDetector::CollisionDetector() { timeline.resize(STEP_COUNT); }

template <PathPointType T>
std::optional<Collision> CollisionDetector::getFirstCollision(std::span<T> path, int robotId, const stp::AvoidObjects& avoidObjects, int timeOffset) const {
    ZoneScopedN("Get First Collision");
    // Collision with objects always takes precedence (i.e. happens BEFORE collision with field or defense area) over field collisions.
    auto collision = getFirstObjectCollision(path, robotId, avoidObjects.shouldAvoidBall, timeOffset);
    if (avoidObjects.shouldAvoidOutOfField && !collision.has_value()) collision = getFirstFieldCollision(path);
    if (avoidObjects.shouldAvoidDefenseArea && !collision.has_value()) collision = getFirstDefenseAreaCollision(path);
    return collision;
}

template <PathPointType T>
bool CollisionDetector::isCollision(const T& pathPoint, const Vector2& obstaclePos, double minDistance) {
    auto distance = (unwrapPosition(pathPoint) - obstaclePos).length();
    return distance < minDistance;
}

void CollisionDetector::updateTimeline(const std::vector<rtt::world::view::RobotView>& robots, const std::optional<rtt::world::view::BallView>& ball) {
    for (int i = 0; i < STEP_COUNT; i++) {
        auto& positionsAtTime = timeline[i];
        positionsAtTime.robotsThem.clear();

        double time = i * (TIME_STEP / 1000.0);
        for (const auto& robot : robots) {
            if (robot->getTeam() == rtt::world::Team::them) {
                positionsAtTime.robotsThem.emplace_back(BB::PosVelVector{robot->getPos() + robot->getVel() * time, robot->getVel()});
                continue;
            }

            if (!PositionControlUtils::isMoving(robot->getPos()) || i == 0) {
                positionsAtTime.robotsUs[robot->getId()] = BB::PosVelVector{robot->getPos(), Vector2{0, 0}};
            }
        }

        if (ball.has_value()) {
            positionsAtTime.ball = BB::PosVelVector{ball->get()->getPos() + ball->get()->getVelocity() * time, ball->get()->getVelocity()};
        } else {
            // TODO: Should we set the ball position to the last known position or somewhere else?
            positionsAtTime.ball = BB::PosVelVector{Vector2{0, 0}, Vector2{0, 0}};
        }
    }
}

void CollisionDetector::setMinBallDistance(double distance) { minBallDistance = distance; }

void CollisionDetector::setField(const std::optional<rtt::world::Field>& newField) { field = newField; }

void CollisionDetector::updateTimelineForOurRobot(std::span<const BB::PosVelVector> path, const Vector2& currentPosition, int robotId) {
    // Sometimes robot did not reach the next step, thus we want to offset the collision by 1.
    // timeline[0] is filed with current position at the start of the tick
    int offset = !path.empty() && !PositionControlUtils::isTargetReached(path[0].position, currentPosition) ? 1 : 0;
    for (int i = 0; i < STEP_COUNT; i++) {
        timeline[i].robotsUs[robotId] = i < path.size() ? path[i] : BB::PosVelVector{currentPosition, Vector2{0, 0}};
    }
}

template <PathPointType T>
bool CollisionDetector::isOccupied(T& position, int robotId, const stp::AvoidObjects& avoidObjects) const {
    auto path = std::vector{unwrapPosition(position)};
    auto collision = getFirstObjectCollision(std::span(path), robotId, avoidObjects.shouldAvoidBall, 0, 1);
    return collision.has_value() && PositionControlUtils::isMoving(collision->velocity);
}

template <PathPointType T>
constexpr const Vector2& CollisionDetector::unwrapPosition(const T& container) {
    // "if constexpr" and "else" is needed, otherwise compiler is confused about the type of the container and errors out.
    // => cannot be reduced to ternary operator
    if constexpr (std::is_same_v<std::decay_t<T>, BB::PosVelVector>) {
        return container.position;
    } else {
        return container;
    }
}

// Explicit instantiation is needed for linking.
template std::optional<Collision> CollisionDetector::getFirstCollision<Vector2>(std::span<Vector2>, int, const stp::AvoidObjects&, int) const;
template std::optional<Collision> CollisionDetector::getFirstCollision<const Vector2>(std::span<const Vector2>, int, const stp::AvoidObjects&, int) const;
template std::optional<Collision> CollisionDetector::getFirstCollision<BB::PosVelVector>(std::span<BB::PosVelVector>, int, const stp::AvoidObjects&, int) const;
template std::optional<Collision> CollisionDetector::getFirstCollision<const BB::PosVelVector>(std::span<const BB::PosVelVector>, int, const stp::AvoidObjects&, int) const;

template bool CollisionDetector::isOccupied<Vector2>(Vector2&, int, const stp::AvoidObjects&) const;

}  // namespace rtt::ai::control
