//
// Created by robzelluf on 10/24/18.
//

#include "TheyHaveBall.h"

namespace rtt {
namespace ai {

//TODO: Fix global namespacing

TheyHaveBall::TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}

bt::Node::Status TheyHaveBall::update() {
    roboteam_msgs::World world = World::get_world();
    std::vector<roboteam_msgs::WorldRobot> robots = world.them;

        bool theyHaveBall = false;
        for(auto &robot : robots) {
            if(coach::Coach::doesRobotHaveBall(robot.id, false)) {
                theyHaveBall = true;
                break;
            }
        }

    if (theyHaveBall) {
        return bt::Node::Status::Success;
    }
    else {
        return bt::Node::Status::Failure;
    }
}

}
}