//
// Created by floris on 15-11-20.
//
#include <include/roboteam_ai/world/WorldData.hpp>
#include <utility>
#include "include/roboteam_ai/world/World.hpp"
#include "control/positionControl/BBTrajectories/WorldObjects.h"

namespace rtt::BB {

    rtt::world::ball::Ball *WorldObjects::ball_;
    std::vector<rtt::world::view::RobotView> WorldObjects::robots;

    WorldObjects::WorldObjects() {

    }

    std::vector<Vector2> WorldObjects::collisionChecker(rtt::BB::BBTrajectory2D BBTrajectory, int robotId) {
        gameState = rtt::ai::GameStateManager::getCurrentGameState();
        ruleset = gameState.getRuleSet();

        double timeStep = 0.1;
        double ballAvoidanceTime = 1;
        auto pathPoints = BBTrajectory.getPathApproach(timeStep);
        std::vector<Vector2> collisions;
        std::vector<double> collisionTimes;

        // If the robot can not move outside the field, check if its path goes outside the field
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

        // If the robot can not move into defense area, check if its path goes into either defense area
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

        // Check if robot is closer to the ball than it is allowed to be
        if (ruleset.minDistanceToBall > 0) {
            auto startPositionBall = ball_->getPos();
            auto VelocityBall = ball_->getFilteredVelocity();
            std::vector<Vector2> ballTrajectory;

            //TODO: improve ball trajectory approximation
            //Current approximation assumes it continues on the same path with the same velocity, and we check 1 second deep
            double time = 0;
            while (pathPoints.size() * timeStep > time || time < ballAvoidanceTime) {
                ballTrajectory.emplace_back(startPositionBall + VelocityBall * time);
                time += timeStep;
            }

            // Check each timeStep for a collision with the ball, or during ball placement if its too close to the 'ballTube'
            auto ballTube = LineSegment(startPositionBall,
                                        Vector2(0, 0));//rtt::ai::GameStateManager::getRefereeDesignatedPosition());
            for (int i = 0; i < ballTrajectory.size(); i++) {
                if (ruleset.minDistanceToBall > (pathPoints[i] - ballTrajectory[i]).length()
                    || (gameState.getStrategyName() == "ball_placement_them"
                        && ruleset.minDistanceToBall > ballTube.distanceToLine(pathPoints[i]))) {
                    collisions.emplace_back(pathPoints[i]);
                    collisionTimes.emplace_back(i * timeStep);
                }
            }
        }
        std::vector<rtt::world::view::RobotView> ourRobots;
        std::vector<rtt::world::view::RobotView> theirRobots;
        for (auto index : robots) {
            if (index->getTeam() == world::us) {
                ourRobots.emplace_back(index);
            } else if (index->getTeam() == world::them) {
                theirRobots.emplace_back(index);
            } else {
                RTT_INFO("A robot has an invalid team, this shouldn't happen.")
            }
        }

        for (int i = 0; i < theirRobots.size()-1; i++) {
            for (int j = 0; j<pathPoints.size()-1; j++) {
                double currentTime = j*timeStep;
                double maxCollisionCheckTime = 0.5;
                if(currentTime <= maxCollisionCheckTime) {
                    auto posDif = BBTrajectory.getPosition(currentTime) - theirRobots[i]->getPos(); //TODO: Change theirRobot position, depending on their velocity. So add their velocity to the position, depending on how far in time you are looking.
                    auto velDif = BBTrajectory.getVelocity(currentTime) - theirRobots[i]->getVel();
                    if (velDif.project2(posDif).length() > 1.5) {
                        if (posDif.length() <= 3 * ai::Constants::ROBOT_RADIUS_MAX()) {
                            //TODO: Make every one of these checks a separate function (like Rolf said yesterday).
                        }
                    }
                }
            }
        }

        if (!collisions.empty()) {
            std::cout << "There is at least 1 collision" << std::endl;
        }
        return collisions;
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

    void WorldObjects::setBall(rtt::world::ball::Ball *ball) {
        ball_ = ball;
    }

    void WorldObjects::setRobots(std::vector<rtt::world::view::RobotView> robots_) {
        robots = std::move(robots_);
    }

}