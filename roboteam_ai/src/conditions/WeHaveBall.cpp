//
// Created by robzelluf on 10/25/18.
//

#include "WeHaveBall.h"
#include "../world/World.h"

namespace rtt {
namespace ai {

WeHaveBall::WeHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
    : Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status WeHaveBall::onUpdate() {
    RobotPtr robotThatHasBall = world::world->whichRobotHasBall();

    if (!robotThatHasBall)
        return bt::Node::Status::Failure;

    if (robotThatHasBall->team == Robot::Team::us)
        return bt::Node::Status::Success;

    return bt::Node::Status::Failure;
}

} // ai
} // rtt