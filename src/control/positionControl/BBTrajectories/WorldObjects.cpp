//
// Created by floris on 15-11-20.
//
#include <include/roboteam_ai/world/WorldData.hpp>
#include <utility>
#include "include/roboteam_ai/world/World.hpp"
#include "control/positionControl/BBTrajectories/WorldObjects.h"

namespace rtt::BB {

    rtt::world::ball::Ball WorldObjects::ball_;
    std::vector<rtt::world::view::RobotView> WorldObjects::robots;

    WorldObjects::WorldObjects() {
        auto ruleset = gameState.getRuleSet();
    };

    std::vector<Vector2> WorldObjects::collisionChecker(rtt::BB::BBTrajectory2D BBTrajectory, int robotId) {
        // You should be able to break before hitting the obstacle if the obstacle is 1.65 seconds away
        // max velocity (8.192) divided by max acceleration (5) is 1.6382 so 1.65s should be safe
        // no collision has to be checked after 1.65s
        double timeStep = 0.1;
        double maxTime = 2;
        auto pathPoints = BBTrajectory.getPathApproach(timeStep);
        std::vector<Vector2> collisions;
        std::vector<double> collisionTimes;

        if (canMoveOutsideField(robotId)) {
            int i = 0;
            for (Vector2 p : pathPoints) {
                if (rtt::ai::FieldComputations::pointIsInField(*field, p, rtt::ai::Constants::ROBOT_RADIUS())) {
                    collisions.emplace_back(p);
                    collisionTimes.emplace_back(i*timeStep);
                }
                i++;
            }
        }

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

        if (ruleset.minDistanceToBall > 0) {
            auto startPositionBall = ball_.getPos();
            auto VelocityBall = ball_.getFilteredVelocity();
            std::vector<Vector2> ballTrajectory;

            //TODO: improve ball trajectory approximation
            //Current approximation just assume it continues on the same path with the same velocity
            double time = 0;
            while (pathPoints.size()*timeStep > time || time > maxTime){
                ballTrajectory.emplace_back(startPositionBall + VelocityBall*time);
                time += timeStep;
            }

            for (int i = 0; i < ballTrajectory.size(); i++) {
                if (ruleset.minDistanceToBall > (pathPoints[i]-ballTrajectory[i]).length()){
                    collisions.emplace_back(pathPoints[i]);
                    collisionTimes.emplace_back(i*timeStep);
                }
            }
        }


        //rtt::ai::Constants::ROBOT_RADIUS()
        //auto robots[0]->getPos()


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

    void WorldObjects::setBall(rtt::world::ball::Ball ball) {
        ball_ = ball;
    }

    void WorldObjects::setRobotPositions(std::vector<rtt::world::view::RobotView> robots_){
        robots = std::move(robots_);
    }

}