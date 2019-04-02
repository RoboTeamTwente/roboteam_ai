//
// Created by robzelluf on 1/22/19.
//

#include <roboteam_ai/src/coach/PassCoach.h>
#include "IsBeingPassedTo.h"

namespace rtt {
namespace ai {

IsBeingPassedTo::IsBeingPassedTo(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

IsBeingPassedTo::Status IsBeingPassedTo::onUpdate() {
    if (coach::g_pass.getRobotBeingPassedTo() == static_cast<int>(robot->id)) {
        return Status::Success;
    }
    else {
        return Status::Failure;
    }
}

std::string IsBeingPassedTo::node_name() {return "IsBeingPassedTo";}

}
}

