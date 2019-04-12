//
// Created by rolf on 12-4-19.
//

#include "IsDefenderGettingBall.h"
#include "../coach/defence/DefenceDealer.h"
namespace rtt {
namespace ai {
IsDefenderGettingBall::IsDefenderGettingBall(string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {
}

std::string IsDefenderGettingBall::node_name() { return "IsDefenderGettingBall"; }

IsDefenderGettingBall::Status IsDefenderGettingBall::onUpdate() {
    coach::g_DefenceDealer.checkIfWeShouldGetBall();
    if (! robot || ! ball) {
        return Status::Waiting;
    }
    if (coach::g_DefenceDealer.isBotGettingBall(robot->id)) {
        return Status::Success;
    }
    return Status::Failure;
}
}//ai
}//rtt

