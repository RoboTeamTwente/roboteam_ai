//
// Created by robzelluf on 4/25/19.
//

#include "include/roboteam_ai/conditions/IsOnPassLine.h"
#include <include/roboteam_ai/world/World.h>
#include <include/roboteam_ai/world/Ball.h>

#include <include/roboteam_ai/control/ControlUtils.h>

namespace rtt {
namespace ai {

IsOnPassLine::IsOnPassLine(std::string name, bt::Blackboard::Ptr blackboard)
    :Condition(std::move(name), std::move(blackboard)) { };

IsOnPassLine::Status IsOnPassLine::onUpdate() {
    int receiverID = coach::g_pass.getRobotBeingPassedTo();
    int passerID = coach::g_pass.getRobotPassing();

    if (receiverID != -1 && passerID != robot->id) {
        RobotPtr receiver = world::world->getRobotForId(receiverID, true);
        if (receiver && control::ControlUtils::isPointProjectedOnLineSegment(robot->pos, ball->pos, receiver->pos)) {
            Vector2 projection = robot->pos.project(ball->pos, receiver->pos);
            if ((projection - robot->pos).length() < DISTANCE_FROM_PASS_LINE) {
                return Status::Success;
            }
        }
    }

    return Status::Failure;
}

}
}