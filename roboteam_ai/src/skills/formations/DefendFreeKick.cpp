//
// Created by baris on 24-4-19.
//

#include <roboteam_ai/src/control/PositionUtils.h>
#include "DefendFreeKick.h"
namespace rtt {
namespace ai {

std::vector<Vector2> DefendFreeKick::posses;
std::shared_ptr<vector<std::shared_ptr<rtt::ai::world::Robot>>> rtt::ai::DefendFreeKick::robotsInFormation = nullptr;


Vector2 DefendFreeKick::getFormationPosition() {
    gtp.setAvoidBall(0.55);

    update = true;
    posses = rtt::ai::control::PositionUtils::getDefendFreeKick(robotsInFormation->size());
    std::vector<int> robotIds;

    for (auto & i : *robotsInFormation) {
        robotIds.push_back(i->id);
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, posses);
    return shortestDistances[robot->id];
}
shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> DefendFreeKick::robotsInFormationPtr() {
    return robotsInFormation;
}

DefendFreeKick::DefendFreeKick(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {
    robotsInFormation = std::make_shared<vector<std::shared_ptr<world::Robot>>>();

}
void DefendFreeKick::onTerminate(Skill::Status s) {
    Formation::onTerminate(s);
    update = false;
}
}

}
