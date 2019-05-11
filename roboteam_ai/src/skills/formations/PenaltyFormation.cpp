//
// Created by baris on 15-4-19.
//

#include <roboteam_ai/src/control/PositionUtils.h>
#include "PenaltyFormation.h"

std::shared_ptr<vector<std::shared_ptr<rtt::ai::world::Robot>>> rtt::ai::PenaltyFormation::robotsInFormation = nullptr;

rtt::ai::PenaltyFormation::PenaltyFormation(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {
    robotsInFormation = std::make_shared<vector<std::shared_ptr<world::Robot>>>();

}
Vector2 rtt::ai::PenaltyFormation::getFormationPosition() {
    if(properties->getBool("Offensive")) {
        // first we calculate all the positions for the defense
        std::vector<int> robotIds;
        for (auto & i : *robotsInFormation) {
            robotIds.push_back(i->id);
        }
        auto poses = rtt::ai::control::PositionUtils::getPenaltyPositions(robotsInFormation->size());

        rtt::HungarianAlgorithm hungarian;
        auto shortestDistances = hungarian.getRobotPositions(robotIds, true, poses);
        return shortestDistances.at(robot->id);
    } else {
        // gtp.setAvoidBall(0.4);
        std::vector<int> robotIds;
        for (auto & i : *robotsInFormation) {
            robotIds.push_back(i->id);
        }
        auto poses = rtt::ai::control::PositionUtils::getDefendPenaltyPositions(robotsInFormation->size());

        rtt::HungarianAlgorithm hungarian;
        auto shortestDistances = hungarian.getRobotPositions(robotIds, true, poses);
        return shortestDistances.at(robot->id);
    }
}

shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> rtt::ai::PenaltyFormation::robotsInFormationPtr() {
    return robotsInFormation;
}
