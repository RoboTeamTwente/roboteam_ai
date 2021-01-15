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



    std::optional<CollisionData> WorldObjects::getFirstCollision(rtt::BB::BBTrajectory2D BBTrajectory, int robotId) {
        //TODO: write a struct to with collision and drivingdirection

        gameState = rtt::ai::GameStateManager::getCurrentGameState();
        ruleset = gameState.getRuleSet();

        double timeStep = 0.1;
        auto pathPoints = BBTrajectory.getPathApproach(timeStep);

        Vector2 drivingDirection;
        std::vector<Vector2> collisions;
        std::vector<double> collisionTimes;

        // If the robot can not move outside the field, check if its path goes outside the field
        this->calculateFieldCollisions(collisions, collisionTimes, drivingDirection, pathPoints, robotId, timeStep);

        // If the robot can not move into defense area, check if its path goes into either defense area
        this->calculateDefenseAreaCollisions(collisions, collisionTimes, drivingDirection, pathPoints, robotId, timeStep);

        // Check if robot is closer to the ball than it is allowed to be
        this->calculateBallCollisions(collisions, collisionTimes, drivingDirection, pathPoints, timeStep);

        // Loop through all pathPoints for each enemy robot, and check if a point in the path will collide with an enemy robot
        this->calculateEnemyRobotCollisions(BBTrajectory, collisions, collisionTimes, drivingDirection, pathPoints, timeStep);

        // For each path already calculated, check if this path collides with those paths
        this->calculateOurRobotCollisions(collisions, collisionTimes, drivingDirection, pathPoints, robotId, timeStep);

        //collisions.emplace_back(Vector2(0,0));
        //collisionTimes.emplace_back(0.1);
        //drivingDirection = pathPoints[3]-pathPoints[2];

        if(!collisionTimes.empty()) {
            double timeOfCollision = *min_element(collisionTimes.begin(), collisionTimes.end());
            auto iterator = find(collisionTimes.begin(), collisionTimes.end(), timeOfCollision);
            int indexOfCollision = iterator - collisionTimes.begin();

            return CollisionData{.collisionPosition = collisions[indexOfCollision],.drivingDirection = drivingDirection.normalize()};
        } else { return std::nullopt; }
    }

    void WorldObjects::calculateFieldCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes, Vector2 &drivingDirection,
                                                const std::vector<Vector2> &pathPoints, int robotId, double timeStep) {
        if (!canMoveOutsideField(robotId)) {
            for (int i = 0; i<pathPoints.size(); i++) {
                if (!rtt::ai::FieldComputations::pointIsInField(*field, pathPoints[i], rtt::ai::Constants::ROBOT_RADIUS())) {
                    collisions.emplace_back(pathPoints[i]);
                    collisionTimes.emplace_back(i * timeStep);
                    drivingDirection = pathPoints[i] - pathPoints[i-1];
                    return;
                }
            }
        }
    }

    void
    WorldObjects::calculateDefenseAreaCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes, Vector2 &drivingDirection,
                                                 const std::vector<Vector2> &pathPoints, int robotId, double timeStep) {
        if (!canEnterDefenseArea(robotId)) {
            for (int i = 0; i<pathPoints.size(); i++) {
                if (rtt::ai::FieldComputations::pointIsInDefenseArea(*field, pathPoints[i], true, 0) ||
                    rtt::ai::FieldComputations::pointIsInDefenseArea(*field, pathPoints[i], false,
                                                                     0.2 + rtt::ai::Constants::ROBOT_RADIUS())) {
                    collisions.emplace_back(pathPoints[i]);
                    collisionTimes.emplace_back(i * timeStep);
                    drivingDirection = pathPoints[i] - pathPoints[i-1];
                    return;
                }
            }
        }
    }

    void WorldObjects::calculateBallCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes, Vector2 &drivingDirection,
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
                    collisions.emplace_back(ballTrajectory[i]);
                    collisionTimes.emplace_back(i * timeStep);
                    drivingDirection = pathPoints[i] - pathPoints[i-1];
                    return;
                }
            }
        }
    }

    void
    WorldObjects::calculateEnemyRobotCollisions(rtt::BB::BBTrajectory2D BBTrajectory, std::vector<Vector2> &collisions,
                                                std::vector<double> &collisionTimes, Vector2 &drivingDirection,
                                                const std::vector<Vector2> &pathPoints,
                                                double timeStep) {
        auto theirRobots = world->getWorld()->getThem();

        for (int j = 0; j < pathPoints.size(); j++) {
            for (int i = 0; i < theirRobots.size() - 1; i++) {
                double currentTime = j * timeStep;
                double maxCollisionCheckTime = 2;
                if (currentTime <= maxCollisionCheckTime) {
                    // TODO: Currently enemy position in future is calculated with simple model (x = x + v*t), maybe improve?
                    Vector2 theirVel = theirRobots[i]->getVel();
                    Vector2 theirPos = theirRobots[i]->getPos() + theirVel * currentTime;
                    Vector2 posDif = BBTrajectory.getPosition(currentTime) -
                                     theirPos; //TODO: Is currentTime defined the same way as the time from the BBTrajectory time?
                    if (posDif.length() < 3 /*<-- kawaiii*/ * ai::Constants::ROBOT_RADIUS_MAX()) {
                        Vector2 ourVel = BBTrajectory.getVelocity(currentTime);
                        Vector2 velDif = ourVel - theirVel;
                        double projectLength = velDif.dot(posDif) / sqrt(posDif.dot(posDif));
                        if (abs(projectLength) > 1.5 && theirVel.length() < ourVel.length()) {
                            collisions.emplace_back(theirPos);
                            collisionTimes.emplace_back(currentTime);
                            drivingDirection = pathPoints[j] - pathPoints[j-1];
                            return;
                        }
                    }
                }
            }
        }
    }

    void
    WorldObjects::calculateOurRobotCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes, Vector2 &drivingDirection,
                                              const std::vector<Vector2> &pathPoints, int robotId, double timeStep) {

        int ourRobotAmount = world->getWorld()->getUs().size();

        for (int j = 0; j < pathPoints.size(); j++) {
            for (int i = 0; i < ourRobotAmount; i++) {
                if (robotId != i && !calculatedPaths[i].empty()) {
                    if ((pathPoints[j] - calculatedPaths[i][j]).length() < ai::Constants::ROBOT_RADIUS() * 1.5 &&
                        j * timeStep < 1) {
                        collisions.emplace_back(calculatedPaths[i][j]);
                        collisionTimes.emplace_back(j * timeStep);
                        drivingDirection = pathPoints[j] - pathPoints[j-1];
                        return;
                    }
                }
            }
        }
    }

    void WorldObjects::storeCalculatedPath(std::vector<Vector2> points, int robotId) {
        calculatedPaths[robotId] = points;
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