//
// Created by robzelluf on 10/24/18.
//

#include "TheyHaveBall.h"
#include "../utilities/Coach.h"

namespace rtt {
namespace ai {

TheyHaveBall::TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}

bt::Node::Status TheyHaveBall::onUpdate() {
    auto w = world::world->getWorld();
    std::vector<Robot> robots = w.them;

        bool theyHaveBall = false;
        for(auto &robot : robots) {
            if(world::world->robotHasBall(robot.id,false)) {
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

} // ai
} // rtt