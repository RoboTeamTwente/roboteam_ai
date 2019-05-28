//
// Created by baris on 23-4-19.
//

#include <roboteam_ai/src/control/PositionUtils.h>
#include <roboteam_ai/src/control/Hungarian.h>
#include "FreeKickFormation.h"

namespace rtt {
namespace ai {

std::vector<Vector2> FreeKickFormation::posses;
std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> rtt::ai::FreeKickFormation::robotsInFormation = nullptr;


Vector2 FreeKickFormation::getFormationPosition() {

    update = true;
    posses = rtt::ai::control::PositionUtils::getFreeKickPositions(robotsInFormation->size());
    std::vector<int> robotIds;

    for (auto & i : *robotsInFormation) {
        robotIds.push_back(i->id);
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, posses);
    return shortestDistances[robot->id];
}
std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> FreeKickFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

FreeKickFormation::FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<bt::Leaf::RobotPtr>>();

};
void FreeKickFormation::onTerminate(Skill::Status s) {
    Formation::onTerminate(s);
    update = false;
}
}

}
