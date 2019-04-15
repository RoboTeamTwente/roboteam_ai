//
// Created by roboteam on 15-4-19.
//

#include "PenaltyFormation.h"
rtt::ai::PenaltyFormation::PenaltyFormation(std::string name, bt::Blackboard::Ptr blackboard)
        :EnterFormation(name, blackboard) {

}
Vector2 rtt::ai::PenaltyFormation::getFormationPosition() {
    return EnterFormation::getFormationPosition();
}
