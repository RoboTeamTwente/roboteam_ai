//
// Created by floris on 15-11-20.
//
#include "control/positionControl/BBTrajectories/WorldObjects.h"

#include <algorithm>
#include <utility>

#include "world/World.hpp"
#include "world/WorldData.hpp"

namespace rtt::BB {

WorldObjects::WorldObjects() = default;

std::optional<CollisionData> WorldObjects::getFirstCollision(const rtt::world::World *world, const rtt::world::Field &field, const Trajectory2D &Trajectory,
                                                             const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId, ai::stp::AvoidObjects avoidObjects) {
    gameState = rtt::ai::GameStateManager::getCurrentGameState();
    ruleset = gameState.getRuleSet();
    // TODO: return the kind of collision
    //^-Question from Max not high priority
    // TODO: find a good value for the timeStep
    double timeStep = 0.1;
    auto pathPoints = Trajectory.getPathApproach(timeStep);

    std::vector<CollisionData> collisionDatas;

    // If the robot can not move outside the field, check if its path goes outside the field
    if (avoidObjects.shouldAvoidOutOfField) {
        calculateFieldCollisions(field, collisionDatas, pathPoints, robotId, timeStep);
    }

    // If the robot can not move into defense area, check if its path goes into either defense area. Don't check if the robot is in the defense are.
    if (avoidObjects.shouldAvoidDefenseArea) {
        calculateDefenseAreaCollisions(field, collisionDatas, pathPoints, robotId, timeStep);
    }

    // Check if robot is closer to the ball than it is allowed to be
    if (avoidObjects.shouldAvoidBall) {
        calculateBallCollisions(world, collisionDatas, pathPoints, timeStep);
    }

    // Loop through all pathPoints for each enemy robot, and check if a point in the path will collide with an enemy robot
    calculateEnemyRobotCollisions(world, Trajectory, collisionDatas, pathPoints, timeStep);

    // For each path already calculated, check if this path collides with those paths
    calculateOurRobotCollisions(world, collisionDatas, pathPoints, computedPaths, robotId, timeStep);

    if (!collisionDatas.empty()) {
        return collisionDatas[0];
    } else {
        return std::nullopt;
    }
}

void WorldObjects::calculateFieldCollisions(const rtt::world::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints, int robotId,
                                            double timeStep) {
    if (!canMoveOutsideField(robotId)) {
        for (int i = 0; i < pathPoints.size(); i++) {
            if (!rtt::ai::FieldComputations::pointIsInField(field, pathPoints[i], rtt::ai::Constants::ROBOT_RADIUS())) {
                insertCollisionData(collisionDatas, CollisionData{pathPoints[i], pathPoints[i], i * timeStep, "FieldCollision"});
                return;
            }
        }
    }
}

void WorldObjects::calculateDefenseAreaCollisions(const rtt::world::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints, int robotId,
                                                  double timeStep) {
    if (!canEnterDefenseArea(robotId)) {
        for (int i = 0; i < pathPoints.size(); i++) {
            if (rtt::ai::FieldComputations::pointIsInOurDefenseArea(field, pathPoints[i]) ||
                rtt::ai::FieldComputations::pointIsInTheirDefenseArea(field, pathPoints[i], 0.2 + rtt::ai::Constants::ROBOT_RADIUS(), 0.2 + rtt::ai::Constants::ROBOT_RADIUS())) {
                insertCollisionData(collisionDatas, CollisionData{pathPoints[i], pathPoints[i], i * timeStep, "DefenseAreaCollision"});
                return;
            }
        }
    }
}

void WorldObjects::calculateBallCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, std::vector<Vector2> pathPoints, double timeStep) {
    if (ruleset.minDistanceToBall > 0) {
        auto startPositionBall = world->getWorld()->getBall()->get()->getPos();
        auto VelocityBall = world->getWorld()->getBall()->get()->getFilteredVelocity();
        std::vector<Vector2> ballTrajectory;

        // TODO: improve ball trajectory approximation
        // Current approximation assumes it continues on the same path with the same velocity, and we check 1 second deep
        double time = 0;
        double ballAvoidanceTime = 1;
        while (pathPoints.size() * timeStep > time && time < ballAvoidanceTime) {
            ballTrajectory.emplace_back(startPositionBall + VelocityBall * time);
            time += timeStep;
        }

        // Check each timeStep for a collision with the ball, or during ball placement if its too close to the 'ballTube'
        auto ballTube = LineSegment(startPositionBall, rtt::ai::GameStateManager::getRefereeDesignatedPosition());
        for (int i = 0; i < ballTrajectory.size(); i++) {
            if (ruleset.minDistanceToBall > (pathPoints[i] - ballTrajectory[i]).length() ||
                (gameState.getStrategyName() == "ball_placement_them" && ruleset.minDistanceToBall > ballTube.distanceToLine(pathPoints[i]))) {
                insertCollisionData(collisionDatas, CollisionData{ballTrajectory[i], pathPoints[i], i * timeStep, "BallCollision"});
                return;
            }
        }
    }
}

void WorldObjects::calculateEnemyRobotCollisions(const rtt::world::World *world, rtt::BB::BBTrajectory2D BBTrajectory, std::vector<CollisionData> &collisionDatas,
                                                 const std::vector<Vector2> &pathPoints, double timeStep) {
    const std::vector<world::view::RobotView> theirRobots = world->getWorld()->getThem();

    for (int i = 0; i < pathPoints.size(); i++) {
        double currentTime = i * timeStep;
        // The >= 2 is used for checking for collisions within 2 seconds
        // TODO: fine tune maximum collision check time
        if (currentTime >= 2) break;
        Vector2 ourVel = Trajectory.getVelocity(currentTime);
        for (const auto &theirRobot : theirRobots) {
            Vector2 theirVel = theirRobot->getVel();
            // TODO: improve position prediction. Current model uses x + v*t
            Vector2 theirPos = theirRobot->getPos() + theirVel * currentTime;
            Vector2 posDif = Trajectory.getPosition(currentTime) - theirPos;
            // TODO: fine tune avoidance distance
            if (posDif.length() < 3 * ai::Constants::ROBOT_RADIUS_MAX()) {
                // Vector2 velDif = ourVel - theirVel;
                // double projectLength = velDif.dot(posDif) / posDif.length();
                // TODO: fine tune allowed speed difference
                // if (abs(projectLength) > 1.5 && theirVel.length() < ourVel.length()) {
                insertCollisionData(collisionDatas, CollisionData{theirPos, pathPoints[i], i * timeStep, "EnemyRobotCollision"});
                return;
                //}
            }
        }
    }
}

void WorldObjects::calculateOurRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                               const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId, double timeStep) {
    auto ourRobots = world->getWorld()->getUs();
    for (int i = 0; i < pathPoints.size(); i++) {
        if (i * timeStep > 2) break;  // Only check for collisions with our robots in the first 2 seconds of our trajectory
        for (auto &robot : ourRobots) {
            if (robotId != robot->getId()) {
                if (computedPaths.find(robot->getId()) != computedPaths.end() && computedPaths.at(robot->getId()).size() > i) {
                    if ((pathPoints[i] - computedPaths.at(robot->getId())[i]).length() < ai::Constants::ROBOT_RADIUS() * 3 /*1.5*/) {
                        insertCollisionData(collisionDatas, CollisionData{computedPaths.at(robot->getId())[i], pathPoints[i], i * timeStep, "OurRobotCollision"});
                        return;
                    }
                } else {
                    if ((pathPoints[i] - robot->getPos()).length() < ai::Constants::ROBOT_RADIUS() * 3 /*1.5*/) {
                        insertCollisionData(collisionDatas, CollisionData{robot->getPos(), pathPoints[i], i * timeStep, "OurRobotCollision"});
                        return;
                    }
                }
            }
        }
    }
}

void WorldObjects::insertCollisionData(std::vector<CollisionData> &collisionDatas, const CollisionData &collisionData) {
    collisionDatas.insert(std::upper_bound(collisionDatas.begin(), collisionDatas.end(), collisionData,
                                           [](CollisionData const &data, CollisionData const &compare) { return data.collisionTime < compare.collisionTime; }),
                          collisionData);
}
}  // namespace rtt::BB