//
// Created by mrlukasbos on 6-12-18.
//

#include "Halt.h"

namespace rtt {
namespace ai {

Halt::Halt(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

Halt::Status Halt::onUpdate() {
    // send empty cmd
    roboteam_msgs::RobotCommand cmd;
    cmd.id = robot->id;
    cmd.x_vel = 0;
    cmd.y_vel = 0;
    cmd.w = 0;
    publishRobotCommand(cmd);

    return Status::Success;
}

} // ai
} // rtt