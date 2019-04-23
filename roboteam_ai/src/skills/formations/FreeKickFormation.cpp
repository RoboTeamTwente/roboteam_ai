//
// Created by baris on 23-4-19.
//

#include "FreeKickFormation.h"

Vector2 rtt::ai::FreeKickFormation::getFormationPosition() {
    return Vector2();
}
shared_ptr<vector<shared_ptr<bt::Leaf::Robot>>> rtt::ai::FreeKickFormation::robotsInFormationPtr() {
    return robotsInFormation;
}
rtt::ai::FreeKickFormation::FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard)
        :Formation(name, blackboard) {

}
