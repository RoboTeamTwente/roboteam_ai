//
// Created by robzelluf on 10/25/18.
//

#include "WeHaveBall.h"
#include "../utilities/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"

namespace rtt {
namespace ai {

WeHaveBall::WeHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
    : Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status WeHaveBall::update() {
    roboteam_msgs::World world = World::get_world();
    std::vector<roboteam_msgs::WorldRobot> robots = world.us;

    bool WeHaveBall = false;
    for (auto &robot : robots) {
        if (World::robotHasBall(robot, *World::getBall())) {
            WeHaveBall = true;
            break;
        }
    }

    if (WeHaveBall) {
        return bt::Node::Status::Success;
    }
    else {
        return bt::Node::Status::Failure;
    }
}

}
}