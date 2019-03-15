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
    if (coach::Coach::getRobotBeingPassedTo() == static_cast<int>(getRobotFromProperties(properties)->id)) {
        return Status::Success;
    }
    else {
        return Status::Failure;
    }
}

std::string IsBeingPassedTo::node_name() {return "IsBeingPassedTo";}

}
}

