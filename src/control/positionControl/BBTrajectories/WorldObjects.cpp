//
// Created by floris on 15-11-20.
//
#include <include/roboteam_ai/world/WorldData.hpp>
#include <utility>
#include "include/roboteam_ai/world/World.hpp"
#include "control/positionControl/BBTrajectories/WorldObjects.h"

namespace rtt::BB {

    world::World *WorldObjects::world;

    WorldObjects::WorldObjects() {

    }

    std::vector<Vector2> WorldObjects::collisionChecker(rtt::BB::BBTrajectory2D BBTrajectory, int robotId) {
        gameState = rtt::ai::GameStateManager::getCurrentGameState();
        ruleset = gameState.getRuleSet();

        double timeStep = 0.1;
        auto pathPoints = BBTrajectory.getPathApproach(timeStep);

        std::vector<Vector2> collisions;
        std::vector<Vector2> &collisionsPtr = collisions;
        std::vector<double> collisionTimes;
        std::vector<double> &collisionTimesPtr = collisionTimes;

        // If the robot can not move outside the field, check if its path goes outside the field
        this->calculateFieldCollisions(collisionsPtr, collisionTimesPtr, pathPoints, robotId, timeStep);

        // If the robot can not move into defense area, check if its path goes into either defense area
        this->calculateDefenseAreaCollisions(collisionsPtr, collisionTimesPtr, pathPoints, robotId, timeStep);

        // Check if robot is closer to the ball than it is allowed to be
        this->calculateBallCollisions(collisionsPtr, collisionTimesPtr, pathPoints, timeStep);

        // Loop through all pathPoints for each enemy robot, and check if a point in the path will collide with an enemy robot
        this->calculateEnemyRobotCollisions(BBTrajectory, collisionsPtr, collisionTimesPtr, pathPoints, timeStep);

        // For each path already calculated, check if this path collides with those paths
        this->calculateOurRobotCollisions(collisionsPtr, collisionTimesPtr, pathPoints, robotId, timeStep);

        return collisions;
    }

    void WorldObjects::calculateFieldCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes,
                                                const std::vector<Vector2> &pathPoints, int robotId, double timeStep) {
        if (!canMoveOutsideField(robotId)) {
            int i = 0;
            for (Vector2 p : pathPoints) {
                if (!rtt::ai::FieldComputations::pointIsInField(*field, p, rtt::ai::Constants::ROBOT_RADIUS())) {
                    collisions.emplace_back(p);
                    collisionTimes.emplace_back(i * timeStep);
                }
                i++;
            }
        }
    }

    void
    WorldObjects::calculateDefenseAreaCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes,
                                                 const std::vector<Vector2> &pathPoints, int robotId, double timeStep) {
        if (!canEnterDefenseArea(robotId)) {
            int i = 0;
            for (Vector2 p : pathPoints) {
                if (rtt::ai::FieldComputations::pointIsInDefenseArea(*field, p, true, 0) ||
                    rtt::ai::FieldComputations::pointIsInDefenseArea(*field, p, false,
                                                                     0.2 + rtt::ai::Constants::ROBOT_RADIUS())) {
                    collisions.emplace_back(p);
                    collisionTimes.emplace_back(i * timeStep);
                }
                i++;
            }
        }
    }

    void WorldObjects::calculateBallCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes,
                                               std::vector<Vector2> pathPoints, double timeStep) {
        if (ruleset.minDistanceToBall > 0) {
            auto startPositionBall = world->getWorld()->getBall()->get()->getPos();
            auto VelocityBall = world->getWorld()->getBall()->get()->getFilteredVelocity();
            std::vector<Vector2> ballTrajectory;

            //TODO: improve ball trajectory approximation
            //Current approximation assumes it continues on the same path with the same velocity, and we check 1 second deep
            double time = 0;
            double ballAvoidanceTime = 1;
            while (pathPoints.size() * timeStep > time || time < ballAvoidanceTime) {
                ballTrajectory.emplace_back(startPositionBall + VelocityBall * time);
                time += timeStep;
            }

            // Check each timeStep for a collision with the ball, or during ball placement if its too close to the 'ballTube'
            auto ballTube = LineSegment(startPositionBall, rtt::ai::GameStateManager::getRefereeDesignatedPosition());
            for (int i = 0; i < ballTrajectory.size(); i++) {
                if (ruleset.minDistanceToBall > (pathPoints[i] - ballTrajectory[i]).length()
                    || (gameState.getStrategyName() == "ball_placement_them"
                        && ruleset.minDistanceToBall > ballTube.distanceToLine(pathPoints[i]))) {
                    collisions.emplace_back(pathPoints[i]);
                    collisionTimes.emplace_back(i * timeStep);
                }
            }
        }
    }

    void
    WorldObjects::calculateEnemyRobotCollisions(rtt::BB::BBTrajectory2D BBTrajectory, std::vector<Vector2> &collisions,
                                                std::vector<double> &collisionTimes, std::vector<Vector2> pathPoints,
                                                double timeStep) {
        auto theirRobots = world->getWorld()->getThem();

        for (int i = 0; i < theirRobots.size() - 1; i++) {
            for (int j = 0; j < pathPoints.size() - 1; j++) {
                double currentTime = j * timeStep;
                double maxCollisionCheckTime = 0.5;
                if (currentTime <= maxCollisionCheckTime) {
                    // TODO: Currently enemy position in future is calculated with simple model (x = x + v*t), maybe improve?
                    Vector2 posDif = BBTrajectory.getPosition(currentTime) - (theirRobots[i]->getPos() +
                                                                              theirRobots[i]->getVel() *
                                                                              currentTime); //TODO: Is currentTime defined the same way as the time from the BBTrajectory time?
                    if (posDif.length() < 3 /*<-- kawaiii*/ * ai::Constants::ROBOT_RADIUS_MAX()) {
                        Vector2 ourVel = BBTrajectory.getVelocity(currentTime);
                        Vector2 theirVel = theirRobots[i]->getVel();
                        Vector2 velDif = ourVel - theirVel;
                        double projectLength = velDif.dot(posDif) / sqrt(posDif.dot(posDif));
                        if (abs(projectLength) > 1.5 && theirVel.length() < ourVel.length()) {
                            collisions.emplace_back(pathPoints[i]);
                            collisionTimes.emplace_back(currentTime);
                        }
                    }
                }
            }
        }
    }

    void
    WorldObjects::calculateOurRobotCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes,
                                              const std::vector<Vector2> &pathPoints, int robotId, double timeStep) {
        for (int i = 0; i < world->getWorld()->getUs().size(); i++) {
            if (!calculatedPaths[i].empty() && robotId != i) {
                for (int j = 0; j < pathPoints.size(); j++) {
                    if ((pathPoints[j] - (calculatedPaths[i])[j]).length() < ai::Constants::ROBOT_RADIUS() * 1.5) {
                        collisions.emplace_back(pathPoints[j]);
                        collisionTimes.emplace_back(j * timeStep);
                    }
                }
            }
        }
    }

    void WorldObjects::storeCalculatedPath(std::vector<Vector2> *points, int robotId) {
        calculatedPaths[robotId] = *points;
    }

    void WorldObjects::setField(const rtt::ai::rtt_world::Field &field_) { this->field = &field_; }

    bool WorldObjects::canEnterDefenseArea(int robotId) {
        if (robotId != gameState.keeperId) {
            return gameState.getRuleSet().robotsCanEnterDefenseArea();
        }
        return true;
    }

    bool WorldObjects::canMoveOutsideField(int robotId) {
        if (robotId != gameState.keeperId) {
            return gameState.getRuleSet().robotsCanGoOutOfField;
        }
        return true;
    }

    void WorldObjects::setWorld(world::World *world_) {
        world = world_;
    }
}