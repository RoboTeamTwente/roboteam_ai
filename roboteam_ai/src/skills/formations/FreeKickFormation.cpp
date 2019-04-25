//
// Created by baris on 23-4-19.
//

#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include "FreeKickFormation.h"
namespace rtt {
namespace ai {

bool FreeKickFormation::calculated = false;
std::vector<Vector2> FreeKickFormation::posses;
std::shared_ptr<vector<std::shared_ptr<rtt::ai::world::Robot>>> rtt::ai::FreeKickFormation::robotsInFormation = nullptr;


Vector2 FreeKickFormation::getFormationPosition() {

    update = true;
    posses = rtt::ai::coach::g_generalPositionCoach.getFreeKickPositions(robotsInFormation->size());
    std::vector<int> robotIds;

    for (auto & i : *robotsInFormation) {
        robotIds.push_back(i->id);
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, posses);
    return shortestDistances[robot->id];
}
shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> FreeKickFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

FreeKickFormation::FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {
    robotsInFormation = std::make_shared<vector<std::shared_ptr<world::Robot>>>();

}
void FreeKickFormation::onTerminate(Skill::Status s) {
    Formation::onTerminate(s);
}
}

}
