//
// Created by baris on 23-4-19.
//

#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include "FreeKickFormation.h"
namespace rtt {
namespace ai {

bool FreeKickFormation::calculated = false;
std::vector<Vector2> FreeKickFormation::posses;

Vector2 FreeKickFormation::getFormationPosition() {
    if (!calculated) {
        posses = rtt::ai::coach::g_generalPositionCoach.getFreeKickPositions(robotsInFormation->size());
        calculated = true;
    }
    std::vector<int> robotIds;

    for (auto & i : *robotsInFormation) {
        robotIds.push_back(i->id);
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, posses);
    return shortestDistances.at(robot->id);
}
shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> FreeKickFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

FreeKickFormation::FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {
}
}

}
