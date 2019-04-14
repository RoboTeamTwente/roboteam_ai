/*
* Consult the pass coach if the current robot is being passed to. in that case: return SUCCESS. else FAILURE.
*/

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
    return Status::Failure;
}

} // ai
} // rtt

