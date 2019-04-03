//
// Created by robzelluf on 10/24/18.
//

#include "TheyHaveBall.h"
#include "../world/World.h"

namespace rtt {
namespace ai {

TheyHaveBall::TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status TheyHaveBall::onUpdate() {
    RobotPtr robotThatHasBall = world::world->whichRobotHasBall();

    if (!robotThatHasBall)
        return bt::Node::Status::Failure;

    if (robotThatHasBall->team == Robot::Team::them)
        return bt::Node::Status::Success;

    return bt::Node::Status::Failure;
}

} // ai
} // rtt