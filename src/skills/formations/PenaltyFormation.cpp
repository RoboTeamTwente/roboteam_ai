//
// Created by baris on 15-4-19.
//

#include "skills/formations/PenaltyFormation.h"

#include <control/ControlUtils.h>
#include <roboteam_utils/Hungarian.h>
#include <control/PositionUtils.h>

std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> rtt::ai::PenaltyFormation::robotsInFormation = nullptr;

rtt::ai::PenaltyFormation::PenaltyFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<bt::Leaf::RobotPtr>>();
}

Vector2 rtt::ai::PenaltyFormation::getFormationPosition() {
    std::vector<Vector2> positions;

    if (properties->getBool("Offensive")) {
        positions = rtt::ai::control::PositionUtils::getPenaltyPositions(robotsInFormation->size());
    } else {
        robot->getNumtreePosControl()->setAvoidBallDistance(0.4);
        positions = rtt::ai::control::PositionUtils::getDefendPenaltyPositions(robotsInFormation->size());
    }

    return getOptimalPosition(robot->id, *robotsInFormation, positions);
}

std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> rtt::ai::PenaltyFormation::robotsInFormationPtr() { return robotsInFormation; }
