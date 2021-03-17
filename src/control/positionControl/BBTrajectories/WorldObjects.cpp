//
// Created by floris on 15-11-20.
//
#include <include/roboteam_ai/world/WorldData.hpp>
#include <utility>
#include "include/roboteam_ai/world/World.hpp"
#include "control/positionControl/BBTrajectories/WorldObjects.h"

namespace rtt::BB {

    world::World *WorldObjects::world;

    WorldObjects::WorldObjects() = default;



    std::optional<CollisionData> WorldObjects::getFirstCollision(rtt::BB::BBTrajectory2D BBTrajectory, int robotId) {
        gameState = rtt::ai::GameStateManager::getCurrentGameState();
        ruleset = gameState.getRuleSet();
        //TODO: return the kind of collision
        //^-Question from Max not high priority
        //TODO: find a good value for the timeStep
        double timeStep = 0.1;
        auto pathPoints = BBTrajectory.getPathApproach(timeStep);

        std::vector<CollisionData> collisionDatas;
        std::vector<double> collisionTimes;

        // If the robot can not move outside the field, check if its path goes outside the field
        calculateFieldCollisions(collisionDatas, collisionTimes, pathPoints, robotId, timeStep);

        // If the robot can not move into defense area, check if its path goes into either defense area
        calculateDefenseAreaCollisions(collisionDatas, collisionTimes, pathPoints, robotId, timeStep);

        // Check if robot is closer to the ball than it is allowed to be
        calculateBallCollisions(collisionDatas, collisionTimes, pathPoints, timeStep);

        // Loop through all pathPoints for each enemy robot, and check if a point in the path will collide with an enemy robot
        calculateEnemyRobotCollisions(BBTrajectory, collisionDatas, collisionTimes, pathPoints, timeStep);

        // For each path already calculated, check if this path collides with those paths
        calculateOurRobotCollisions(collisionDatas, collisionTimes, pathPoints, robotId, timeStep);

        if(!collisionTimes.empty()) {
            double timeOfCollision = *min_element(collisionTimes.begin(), collisionTimes.end());
            auto iterator = find(collisionTimes.begin(), collisionTimes.end(), timeOfCollision);
            int indexOfCollision = iterator - collisionTimes.begin();

            return collisionDatas[indexOfCollision];
        } else { return std::nullopt; }
    }

    void WorldObjects::calculateFieldCollisions(std::vector<CollisionData> &collisionDatas, std::vector<double> &collisionTimes,
                                                const std::vector<Vector2> &pathPoints, int robotId, double timeStep) {
        if (!canMoveOutsideField(robotId)) {
            for (int i = 0; i<pathPoints.size(); i++) {
                if (!rtt::ai::FieldComputations::pointIsInField(*field, pathPoints[i], rtt::ai::Constants::ROBOT_RADIUS())) {
                    collisionDatas.emplace_back(CollisionData{pathPoints[i], pathPoints[i]});
                    collisionTimes.emplace_back(i * timeStep);
                    return;
                }
            }
        }
    }

    void
    WorldObjects::calculateDefenseAreaCollisions(std::vector<CollisionData> &collisionDatas, std::vector<double> &collisionTimes,
                                                 const std::vector<Vector2> &pathPoints, int robotId, double timeStep) {
        if (!canEnterDefenseArea(robotId)) {
            for (int i = 0; i<pathPoints.size(); i++) {
                if (rtt::ai::FieldComputations::pointIsInDefenseArea(*field, pathPoints[i], true, 0) ||
                    rtt::ai::FieldComputations::pointIsInDefenseArea(*field, pathPoints[i], false,
                                                                     0.2 + rtt::ai::Constants::ROBOT_RADIUS())) {
                    collisionDatas.emplace_back(CollisionData{pathPoints[i], pathPoints[i]});
                    collisionTimes.emplace_back(i * timeStep);
                    return;
                }
            }
        }
    }

    void WorldObjects::calculateBallCollisions(std::vector<CollisionData> &collisionDatas, std::vector<double> &collisionTimes,
                                               std::vector<Vector2> pathPoints, double timeStep) {
        if (ruleset.minDistanceToBall > 0) {
            auto startPositionBall = world->getWorld()->getBall()->get()->getPos();
            auto VelocityBall = world->getWorld()->getBall()->get()->getFilteredVelocity();
            std::vector<Vector2> ballTrajectory;

            //TODO: improve ball trajectory approximation
            //Current approximation assumes it continues on the same path with the same velocity, and we check 1 second deep
            double time = 0;
            double ballAvoidanceTime = 1;
            while (pathPoints.size() * timeStep > time && time < ballAvoidanceTime) {
                ballTrajectory.emplace_back(startPositionBall + VelocityBall * time);
                time += timeStep;
            }

            // Check each timeStep for a collision with the ball, or during ball placement if its too close to the 'ballTube'
            auto ballTube = LineSegment(startPositionBall, rtt::ai::GameStateManager::getRefereeDesignatedPosition());
            for (int i = 0; i < ballTrajectory.size(); i++) {
                if (ruleset.minDistanceToBall > (pathPoints[i] - ballTrajectory[i]).length()
                    || (gameState.getStrategyName() == "ball_placement_them"
                        && ruleset.minDistanceToBall > ballTube.distanceToLine(pathPoints[i]))) {
                    collisionDatas.emplace_back(CollisionData{ballTrajectory[i], pathPoints[i]});
                    collisionTimes.emplace_back(i * timeStep);
                    return;
                }
            }
        }
    }

    void
    WorldObjects::calculateEnemyRobotCollisions(rtt::BB::BBTrajectory2D BBTrajectory, std::vector<CollisionData> &collisionDatas,
                                                std::vector<double> &collisionTimes, const std::vector<Vector2> &pathPoints,
                                                double timeStep) {
        auto theirRobots = world->getWorld()->getThem();

        for (int i = 0; i < pathPoints.size(); i++) {
            for (int j = 0; j < theirRobots.size() - 1; j++) {
                double currentTime = i * timeStep;
                double maxCollisionCheckTime = 2;
                if (currentTime <= maxCollisionCheckTime) {
                    // TODO: Improve enemy position in future, currently is calculated with simple model (x = x + v*t)
                    Vector2 theirVel = theirRobots[j]->getVel();
                    Vector2 theirPos = theirRobots[j]->getPos() /*+ theirVel * currentTime*/;
                    Vector2 posDif = BBTrajectory.getPosition(currentTime) - theirPos;
                    if (posDif.length() < 3 * ai::Constants::ROBOT_RADIUS_MAX()) {
                        Vector2 ourVel = BBTrajectory.getVelocity(currentTime);
                        Vector2 velDif = ourVel - theirVel;
                        double projectLength = velDif.dot(posDif) / sqrt(posDif.dot(posDif));
                        if (abs(projectLength) > 1.5 && theirVel.length() < ourVel.length()) {
                            collisionDatas.emplace_back(CollisionData{theirPos, pathPoints[i]});
                            collisionTimes.emplace_back(currentTime);
                            return;
                        }
                    }
                }
            }
        }
    }

    void
    WorldObjects::calculateOurRobotCollisions(std::vector<CollisionData> &collisionDatas, std::vector<double> &collisionTimes,
                                              const std::vector<Vector2> &pathPoints, int robotId, double timeStep) {

        int ourRobotAmount = world->getWorld()->getUs().size();

        for (int i = 0; i < pathPoints.size(); i++) {
            for (int j = 0; j < ourRobotAmount; j++) {
                if (robotId != j && !calculatedPaths[j].empty()) {
                    if ((pathPoints[i] - calculatedPaths[j][i]).length() < ai::Constants::ROBOT_RADIUS() * 1.5 &&
                        i * timeStep < 1) {
                        collisionDatas.emplace_back(CollisionData{calculatedPaths[j][i], pathPoints[i]});
                        collisionTimes.emplace_back(i * timeStep);
                        return;
                    }
                }
            }
        }
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