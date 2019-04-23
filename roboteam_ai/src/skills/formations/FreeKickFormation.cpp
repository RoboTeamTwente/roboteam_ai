//
// Created by baris on 23-4-19.
//

#include "FreeKickFormation.h"
namespace rtt {
namespace ai {

Vector2 FreeKickFormation::getFormationPosition() {

    if (offensive) {
        Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
        Vector2 penaltyPoint = rtt::ai::world::field->getPenaltyPoint(false);
        Vector2 line = penaltyPoint - ballPos;
        Vector2 targeting = line.stretchToLength(line.length()*(3.0/5.0));
        // TODO: make smarter, fine for now
        return (ballPos + targeting);
    }
    else {
        // -1 because one is offensive
        for (unsigned int i = 0; i < (robotsInFormation->size() -1); i ++) {

        }

    }

}
shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> FreeKickFormation::robotsInFormationPtr() {
    return robotsInFormation;
}
FreeKickFormation::FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {

}
void FreeKickFormation::onInitialize() {
    offensive = properties->getBool("Offensive");
    robotsInFormationMemory = 0;
    addRobotToFormation();}
}
}