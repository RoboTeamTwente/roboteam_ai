//
// Created by baris on 15-4-19.
//

#include "skills/formations/PenaltyFormation.h"
#include <control/ControlUtils.h>
#include <control/Hungarian.h>
#include <control/PositionUtils.h>

std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> rtt::ai::PenaltyFormation::robotsInFormation = nullptr;

rtt::ai::PenaltyFormation::PenaltyFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<bt::Leaf::RobotPtr>>();
}

Vector2 rtt::ai::PenaltyFormation::getFormationPosition() {
    if (properties->getBool("Offensive")) {
        // first we calculate all the positions for the defense
        std::vector<int> robotIds;
        for (auto &i : *robotsInFormation) {
            robotIds.push_back(i->id);
        }
        auto poses = rtt::ai::control::PositionUtils::getPenaltyPositions(*field, robotsInFormation->size());

        rtt::HungarianAlgorithm hungarian;
        auto shortestDistances = hungarian.getRobotPositions(robotIds, true, poses);
        return shortestDistances.at(robot->id);
    } else {
        robot->getNumtreePosControl()->setAvoidBallDistance(0.4);
        std::vector<int> robotIds;
        for (auto &i : *robotsInFormation) {
            robotIds.push_back(i->id);
        }
        auto poses = rtt::ai::control::PositionUtils::getDefendPenaltyPositions(*field, robotsInFormation->size());

        rtt::HungarianAlgorithm hungarian;
        auto shortestDistances = hungarian.getRobotPositions(robotIds, true, poses);
        return shortestDistances.at(robot->id);
    }
}

std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> rtt::ai::PenaltyFormation::robotsInFormationPtr() { return robotsInFormation; }
