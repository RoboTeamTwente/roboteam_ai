//
// Created by floris on 15-11-20.
//
#include <world/WorldData.hpp>
#include "world/World.hpp"
#include "control/positionControl/BBTrajectories/WorldObjects.h"
#include <algorithm>

namespace rtt::BB {

    WorldObjects::WorldObjects() = default;



    std::optional<CollisionData> WorldObjects::getFirstCollision(const rtt::world::World *world, const rtt::world::Field &field,
                                                                 const std::unordered_map<int, std::vector<Vector2>> &computedPaths,
                                                                 const std::unordered_map<int, std::vector<Vector2>> &computedVelocities,
                                                                 std::optional<double> ballAvoidanceDistance,
                                                                 int robotId, const double pathTimeStep, const double velTimeStep) {
        gameState = rtt::ai::GameStateManager::getCurrentGameState();
        ruleset = gameState.getRuleSet();

        std::vector<CollisionData> collisionDatas;

        // If the robot can not move outside the field, check if its path goes outside the field
        calculateFieldCollisions(field, collisionDatas, computedPaths.at(robotId), robotId, pathTimeStep);

        // If the robot can not move into defense area, check if its path goes into either defense area
        calculateDefenseAreaCollisions(field, collisionDatas, computedPaths.at(robotId), robotId, pathTimeStep);

        // Check if robot is closer to the ball than it is allowed to be
        calculateBallCollisions(world, collisionDatas, computedPaths.at(robotId), ballAvoidanceDistance, pathTimeStep);

        // Loop through all pathPoints for each enemy robot, and check if a point in the path will collide with an enemy robot
        calculateEnemyRobotCollisions(world, collisionDatas, computedPaths.at(robotId), computedVelocities.at(robotId), pathTimeStep, velTimeStep);

        // For each path already calculated, check if this path collides with those paths
        calculateOurRobotCollisions(world, collisionDatas, computedPaths.at(robotId), computedPaths, robotId, pathTimeStep);

        if(!collisionDatas.empty()) {
            return collisionDatas[0];
        } else { return std::nullopt; }
    }

    void WorldObjects::calculateFieldCollisions(const rtt::world::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                                int robotId, double timeStep) {
        if (!canMoveOutsideField(robotId)) {
            // Loop through all pathPoints and check if the point is outside the field
            for (int i = 0; i < pathPoints.size(); i++) {
                if (!rtt::ai::FieldComputations::pointIsInField(field, pathPoints[i], rtt::ai::Constants::ROBOT_RADIUS())) {
                    // The obstaclePosition and collisionPosition are saved as the same location
                    insertCollisionData(collisionDatas,CollisionData{pathPoints[i], pathPoints[i], i * timeStep, "FieldCollision"});
                    return;
                }
            }
        }
    }

    void
    WorldObjects::calculateDefenseAreaCollisions(const rtt::world::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                                 int robotId, double timeStep) {
        if (!canEnterDefenseArea(robotId)) {
            // Loop through all pathPoints and check if the point is inside our or their defense area
            for (int i = 0; i < pathPoints.size(); i++) {
                if (rtt::ai::FieldComputations::pointIsInDefenseArea(field, pathPoints[i], true, 0) ||
                    rtt::ai::FieldComputations::pointIsInDefenseArea(field, pathPoints[i], false,
                                                                     0.2 + rtt::ai::Constants::ROBOT_RADIUS())) {
                    // The obstaclePosition and collisionPosition are saved as the same location
                    insertCollisionData(collisionDatas,CollisionData{pathPoints[i], pathPoints[i], i * timeStep, "DefenseAreaCollision"});
                    return;
                }
            }
        }
    }

    void WorldObjects::calculateBallCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas,std::vector<Vector2> pathPoints,
                                               std::optional<double> ballAvoidanceDistance, double timeStep) {
        // The ball should be avoided when ballAvoidanceDistance has no value and the ruleset minDistanceToBall is larger than 0 or
        // when the ballAvoidanceDistance has a value and is larger than 0
        // The ballAvoidanceDistance thus overwrites the ruleset minDistanceToBall!
        if ((!ballAvoidanceDistance.has_value() && ruleset.minDistanceToBall > 0.001) || (ballAvoidanceDistance.has_value() && ballAvoidanceDistance.value() > 0.001)) {
            auto startPositionBall = world->getWorld()->getBall()->get()->getPos();
            auto VelocityBall = world->getWorld()->getBall()->get()->getFilteredVelocity();
            std::vector<Vector2> ballTrajectory;

            //TODO: improve ball trajectory approximation
            //Current approximation assumes it continues on the same path with the same velocity, and we check for 1 second
            double time = 0;
            double ballAvoidanceTime = 1;
            while (pathPoints.size() * timeStep > time && time < ballAvoidanceTime) {
                ballTrajectory.emplace_back(startPositionBall + VelocityBall * time);
                time += timeStep;
            }

            // Check each timeStep for a collision with the ball, or during ball placement if its too close to the 'ballTube'
            auto ballTube = LineSegment(startPositionBall, rtt::ai::GameStateManager::getRefereeDesignatedPosition());
            for (int i = 0; i < ballTrajectory.size(); i++) {
                double pathDistance = (pathPoints[i] - ballTrajectory[i]).length();
                // The first statement is to check if there is a balltube that we can't go into
                // The second and third check if the path is too close close according to the ruleset or ballAvoidanceDistance depending on
                // if ballAvoidanceDistance has a value
                if ((gameState.getStrategyName() == "ball_placement_them" && ruleset.minDistanceToBall > ballTube.distanceToLine(pathPoints[i])) ||
                    (!ballAvoidanceDistance.has_value() && ruleset.minDistanceToBall > pathDistance) ||
                    (ballAvoidanceDistance.has_value() && ballAvoidanceDistance.value() > pathDistance)) {
                    insertCollisionData(collisionDatas,CollisionData{ballTrajectory[i], pathPoints[i], i * timeStep, "BallCollision"});
                    return;
                }
            }
        }
    }

    void
    WorldObjects::calculateEnemyRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                                const std::vector<Vector2> &velocityPoints, double pathTimeStep, double velTimeStep) {
        const std::vector<world::view::RobotView> theirRobots = world->getWorld()->getThem();

        // Loop through all pathPoints
        int maxSteps = velocityPoints.size() * velTimeStep / pathTimeStep;
        for (int i = 0; i < maxSteps; i++) {
            double currentTime = i * pathTimeStep;
            // The >= 2 is used for checking for collisions within 2 seconds
            // TODO: fine tune maximum collision check time
            if (currentTime >= 2) break;
            Vector2 ourVel = velocityPoints.at((int)((double)i * pathTimeStep / velTimeStep));

            // Loop through all enemy robots
            for (const auto &theirRobot : theirRobots) {
                Vector2 theirVel = theirRobot->getVel();

                // Calculate enemy robot prediction
                // TODO: improve position prediction. Current model uses x + v*t
                Vector2 theirPos = theirRobot->getPos() + theirVel * currentTime;
                Vector2 posDif = pathPoints.at(i) - theirPos;

                // TODO: fine tune avoidance distance
                if (posDif.length() < 3 * ai::Constants::ROBOT_RADIUS_MAX()) {
                    // Official crashing rules:
                    // At the moment of collision of two robots of different teams, the difference of the speed vectors of both robots is taken and projected
                    // onto the line that is defined by the position of both robots. If the length of this projection is greater than 1.5 meters per second,
                    // the faster robot committed a foul. If the absolute robot speed difference is less than 0.3 meters per second, both conduct a foul.
                    Vector2 velDif = ourVel - theirVel;
                    double projectLength = velDif.dot(posDif) / posDif.length();
                    // TODO: fine tune allowed speed difference
                    if (abs(projectLength) > 1.5 && theirVel.length() < ourVel.length()) {
                        insertCollisionData(collisionDatas,CollisionData{theirPos, pathPoints[i], i * pathTimeStep, "EnemyRobotCollision"});
                        return;
                    }
                }
            }
        }
    }

    void
    WorldObjects::calculateOurRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas,const std::vector<Vector2> &pathPoints,
                                              const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId, double timeStep) {
        const std::vector<world::view::RobotView> ourRobots = world->getWorld()->getUs();

        // Loop through path points
        for (int i = 0; i < pathPoints.size(); i++) {
            // Loop through our robots
            for (const auto &ourRobot : ourRobots) {
                // Check the paths for collisions that were already calculated except for the path of the current robot
                if (robotId != ourRobot->getId() && computedPaths.find(ourRobot->getId()) != computedPaths.end()) {
                    // Check if the path to check has less points than current i and if so only checks the final position of that robot
                    Vector2 computedPathsPositionToCheck = computedPaths.at(ourRobot->getId()).size() > i ? computedPaths.at(ourRobot->getId())[i] : computedPaths.at(ourRobot->getId()).back();
                    if ((pathPoints[i] - computedPathsPositionToCheck).length() < ai::Constants::ROBOT_RADIUS() * 1.5 && i * timeStep < 1) {
                        insertCollisionData(collisionDatas,CollisionData{computedPaths.at(ourRobot->getId())[i], pathPoints[i], i * timeStep, "OurRobotCollision"});
                        return;
                    }
                }
            }
        }
    }

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

    void WorldObjects::insertCollisionData(std::vector<CollisionData> &collisionDatas, const CollisionData &collisionData) {
        collisionDatas.insert(
            std::upper_bound(collisionDatas.begin(), collisionDatas.end(), collisionData, [](CollisionData const& data, CollisionData const& compare) {
              return data.collisionTime < compare.collisionTime;
            }),
            collisionData
        );
    }
}  // namespace rtt::BB