//
// Created by mrlukasbos on 11-12-18.
//

#include "CanSeeGoal.h"

namespace rtt {
namespace ai {

CanSeeGoal::CanSeeGoal(std::string name, bt::Blackboard::Ptr blackboard)
    : Condition(name, blackboard) { }

CanSeeGoal::Status CanSeeGoal::update() {
    robot = getRobotFromProperties(properties);



    return Status::Success;
}

} // ai
} // rtt