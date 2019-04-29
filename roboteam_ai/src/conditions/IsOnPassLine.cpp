//
// Created by robzelluf on 4/25/19.
//

#include "IsOnPassLine.h"

namespace rtt {
namespace ai {

IsOnPassLine::IsOnPassLine(std::string name, bt::Blackboard::Ptr blackboard)
    :Condition(std::move(name), std::move(blackboard)) { };

IsOnPassLine::Status IsOnPassLine::onUpdate() {
    int receiverID = coach::g_pass.getRobotBeingPassedTo();
    int passerID = coach::g_pass.getRobotPassing();

    if (receiverID != -1 && passerID != robot->id) {
        RobotPtr receiver = world::world->getRobotForId(receiverID, true);
        if (control::ControlUtils::isPointProjectedOnLineSegment(robot->pos, ball->pos, receiver->pos)) {
            return Status::Success;
        }
    }

    return Status::Failure;
}

}
}