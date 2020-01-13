/*
* Return SUCCESS if their robots has the ball, otherwise FAILURE
*/

#include "conditions/TheyHaveBall.h"
#include "world_old/World.h"
#include "world_old/Robot.h"

namespace rtt::ai {

TheyHaveBall::TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status TheyHaveBall::onUpdate() {
    RobotPtr robotThatHasBall = world->whichRobotHasBall();

    if (robotThatHasBall && robotThatHasBall->team == Team::them) {
        return bt::Node::Status::Success;
    }

    return bt::Node::Status::Failure;
}

} // rtt