//
// Created by robzelluf on 10/25/18.
//

#include "WeHaveBall.h"

namespace rtt {
namespace ai {

//TODO: Fix global namespacing

WeHaveBall::WeHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}

bt::Node::Status WeHaveBall::update() {
    roboteam_msgs::World world = World::get_world();
    std::vector<roboteam_msgs::WorldRobot> robots = world.us;

    bool WeHaveBall = false;
    for (auto &robot : robots) {
        if (World::bot_has_ball(robot, World::getBall())) {
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