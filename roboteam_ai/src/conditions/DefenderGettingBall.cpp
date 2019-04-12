//
// Created by rolf on 12-4-19.
//

#include "DefenderGettingBall.h"
#include "../coach/defence/DefenceDealer.h"
namespace rtt{
namespace ai{
DefenderGettingBall::DefenderGettingBall(string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {
}
std::string DefenderGettingBall::node_name() {return "DefenderGettingBall"; }
DefenderGettingBall::Status DefenderGettingBall::onUpdate() {
    if (!robot||!ball){
        return Status::Waiting;
    }
    if (coach::g_DefenceDealer.isBotGettingBall(robot->id)){
        return Status::Success;
    }
    return Status::Failure;
}
}//ai
}//rtt

