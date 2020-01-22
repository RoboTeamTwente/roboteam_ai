//
// Created by robzelluf on 4/25/19.
//

#include "conditions/IsOnPassLine.h"
#include <world/World.h>
#include <world/Ball.h>
#include <control/ControlUtils.h>

namespace rtt {
namespace ai {

    /**
     * Checks if the robot is on the pass line. Initialize this skill with a name corresponding to the role,
     * and a blackboard (that can be empty as of writing this documentation)
     * @param name
     * @param blackboard the blackboard passed into the function from which the skill can be given data
     */
IsOnPassLine::IsOnPassLine(std::string name, bt::Blackboard::Ptr blackboard)
    :Condition(std::move(name), std::move(blackboard)) { };

IsOnPassLine::Status IsOnPassLine::onUpdate() {
    int receiverID = coach::g_pass.getRobotBeingPassedTo();
    int passerID = coach::g_pass.getRobotPassing();

    if (receiverID != -1 && passerID != robot->id) {
        RobotPtr receiver = world->getRobotForId(receiverID, true);
        if (receiver && control::ControlUtils::isPointProjectedOnLineSegment(robot->pos, ball->getPos(), receiver->pos)) {
            Vector2 projection = robot->pos.project(ball->getPos(), receiver->pos);
            if ((projection - robot->pos).length() < DISTANCE_FROM_PASS_LINE) {
                return Status::Success;
            }
        }
    }

    return Status::Failure;
}

}
}