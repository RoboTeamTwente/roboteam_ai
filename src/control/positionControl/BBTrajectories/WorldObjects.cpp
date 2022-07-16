//
// Created by floris on 15-11-20.
//
#include "control/positionControl/BBTrajectories/WorldObjects.h"

#include <algorithm>

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

    // Delete last 3 points of trajectory if not in last 10 points (doesn't check for collision at endpoint)
    if (computedPaths.contains(robotId) && computedPaths.at(robotId).size() > 10 && pathPoints.size() > 10) {
        pathPoints.erase(pathPoints.end() - 3, pathPoints.end());
    }

    size_t timeStepsDone;
    computedPaths.contains(robotId) && pathPoints.size() > computedPaths.at(robotId).size() ? timeStepsDone = pathPoints.size() - computedPaths.at(robotId).size()
                                                                                            : timeStepsDone = 0;

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
        calculateBallCollisions(world, collisionDatas, pathPoints, timeStep, avoidObjects.avoidBallDist);
    }

    // Loop through all pathPoints for each enemy robot, and check if a point in the path will collide with an enemy robot
    if (avoidObjects.shouldAvoidTheirRobots) {
        calculateEnemyRobotCollisions(world, Trajectory, collisionDatas, pathPoints, timeStep, timeStepsDone);
    }

    // For each path already calculated, check if this path collides with those paths
    if (avoidObjects.shouldAvoidOurRobots) {
        calculateOurRobotCollisions(world, collisionDatas, pathPoints, computedPaths, robotId, timeStep, timeStepsDone);
    }

    if (!collisionDatas.empty()) {
        return collisionDatas[0];
    } else {
        return std::nullopt;
    }
}

void WorldObjects::calculateFieldCollisions(const rtt::world::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints, int robotId,
                                            double timeStep) {
    for (size_t i = 0; i < pathPoints.size(); i++) {
        if (!rtt::ai::FieldComputations::pointIsInField(field, pathPoints[i], rtt::ai::Constants::ROBOT_RADIUS())) {
            // Don't care about the field if the robot is already outside the field (i == 0 is the first point of the robot's path, so almost the currentPosition).
            if (i == 0) return;

            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], pathPoints[i], i * timeStep, "FieldCollision"});
            return;
        }
    }
}

void WorldObjects::calculateDefenseAreaCollisions(const rtt::world::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints, int robotId,
                                                  double timeStep) {
    auto ourDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field, true, 0, 0);
    auto theirDefenseArea = rtt::ai::FieldComputations::getDefenseArea(field, false, ai::stp::control_constants::DEFENSE_AREA_AVOIDANCE_MARGIN, 0);

    for (size_t i = 0; i < pathPoints.size(); i++) {
        if (ourDefenseArea.contains(pathPoints[i]) || theirDefenseArea.contains(pathPoints[i])) {
            // Don't care about the defense area if the robot is already in the defense area. It should just get out as fast as possible :)
            if (i == 0) return;

            insertCollisionData(collisionDatas, CollisionData{pathPoints[i], pathPoints[i], i * timeStep, "DefenseAreaCollision"});
            return;
        }
    }
}

void WorldObjects::calculateBallCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, std::vector<Vector2> pathPoints, double timeStep, double dist) {
    auto startPositionBall = world->getWorld()->getBall()->get()->position;
    auto VelocityBall = world->getWorld()->getBall()->get()->velocity;
    std::vector<Vector2> ballTrajectory;

    // TODO: improve ball trajectory approximation
    // Current approximation assumes it continues on the same path with the same velocity, and we check 5 seconds deep
    double time = 0;
    double ballAvoidanceTime = 5;
    while (pathPoints.size() * timeStep > time && time < ballAvoidanceTime) {
        ballTrajectory.emplace_back(startPositionBall + VelocityBall * time);
        time += timeStep;
    }

    for (size_t i = 1; i < ballTrajectory.size(); i++) {
        if (pathPoints[i].dist(ballTrajectory[i]) < dist){
            insertCollisionData(collisionDatas, CollisionData{ballTrajectory[i], pathPoints[i], i * timeStep, "BallCollision"});
            return;
        }
    }
}

void WorldObjects::calculateEnemyRobotCollisions(const rtt::world::World *world, rtt::Trajectory2D Trajectory, std::vector<CollisionData> &collisionDatas,
                                                 const std::vector<Vector2> &pathPoints, double timeStep, size_t timeStepsDone) {
    const std::vector<world::view::RobotView> theirRobots = world->getWorld()->getThem();

    for (size_t i = timeStepsDone; i < pathPoints.size(); i++) {
        double currentTime = i * timeStep;
        // The >= 2 is used for checking for collisions within 2 seconds
        // TODO: fine tune maximum collision check time
        if (currentTime - timeStepsDone * timeStep >= 2) break;
        // Vector2 ourVel = Trajectory.getVelocityBall(currentTime);
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
                                               const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId, double timeStep, size_t timeStepsDone) {
    auto ourRobots = world->getWorld()->getUs();
    for (size_t i = timeStepsDone; i < pathPoints.size(); i++) {
        if (i * timeStep - timeStepsDone * timeStep > 2) break;  // Only check for collisions with our robots in the first 2 seconds of our trajectory
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