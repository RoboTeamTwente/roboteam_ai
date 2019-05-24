/*
* Return SUCCESS if their robots has the ball, otherwise FAILURE
*/

#include "TheyHaveBall.h"
#include "../world/World.h"

namespace rtt {
namespace ai {

TheyHaveBall::TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status TheyHaveBall::onUpdate() {
    RobotPtr robotThatHasBall = world::world->whichRobotHasBall();

    if (robotThatHasBall && robotThatHasBall->team == world::Robot::Team::them) {
        return bt::Node::Status::Success;
    }

    return bt::Node::Status::Failure;
}

} // ai
} // rtt