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
                    collisionTimes.emplace_back(i*timeStep);
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
                    collisionTimes.emplace_back(i*timeStep);
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
            while (pathPoints.size()*timeStep > time || time < ballAvoidanceTime){
                ballTrajectory.emplace_back(startPositionBall + VelocityBall*time);
                time += timeStep;
            }

            // Check each timeStep for a collision with the ball, or during ball placement if its too close to the 'ballTube'
            auto ballTube = LineSegment(startPositionBall,Vector2(0,0));//rtt::ai::GameStateManager::getRefereeDesignatedPosition());
            for (int i = 0; i < ballTrajectory.size(); i++) {
                if (ruleset.minDistanceToBall > (pathPoints[i]-ballTrajectory[i]).length()
                    || (gameState.getStrategyName() == "ball_placement_them"
                        && ruleset.minDistanceToBall > ballTube.distanceToLine(pathPoints[i]))){
                    collisions.emplace_back(pathPoints[i]);
                    collisionTimes.emplace_back(i*timeStep);
                }
            }
        }
        if(!collisions.empty()) {
            std::cout << "There is at least 1 collision" << std::endl;
        }
        return collisions;
    }

    void WorldObjects::setField(const rtt::ai::rtt_world::Field &field) { this->field = &field; }

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

    void WorldObjects::setRobotPositions(std::vector<rtt::world::view::RobotView> robots_){
        robots = std::move(robots_);
    }

}