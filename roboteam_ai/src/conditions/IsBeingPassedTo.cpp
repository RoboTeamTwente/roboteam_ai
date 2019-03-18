//
// Created by robzelluf on 1/22/19.
//

#include "IsBeingPassedTo.h"

namespace rtt {
namespace ai {

IsBeingPassedTo::IsBeingPassedTo(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void IsBeingPassedTo::onInitialize() {};

IsBeingPassedTo::Status IsBeingPassedTo::onUpdate() {
    robot = getRobotFromProperties(properties);
    if (coach::Coach::getRobotBeingPassedTo() == robot->id) {
        return Status::Success;
    }
    else {
        return Status::Failure;
    }
}

std::string IsBeingPassedTo::node_name() {return "IsBeingPassedTo";}

}
}

