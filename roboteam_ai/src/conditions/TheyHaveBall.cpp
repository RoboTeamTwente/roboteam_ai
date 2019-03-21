//
// Created by robzelluf on 10/24/18.
//

#include "TheyHaveBall.h"
#include "../utilities/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"

namespace rtt {
namespace ai {

TheyHaveBall::TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}

bt::Node::Status TheyHaveBall::onUpdate() {
    roboteam_msgs::World world = World::get_world();
    std::vector<roboteam_msgs::WorldRobot> robots = world.them;

        bool theyHaveBall = false;
        for(auto &robot : robots) {
            if(World::botHasBall(robot.id,false)) {
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