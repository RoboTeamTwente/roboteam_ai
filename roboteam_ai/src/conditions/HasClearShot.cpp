//
// Created by robzelluf on 3/14/19.
//

#include "HasClearShot.h"

namespace rtt{
namespace ai {

HasClearShot::HasClearShot(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {};

void HasClearShot::onInitialize() {};

HasClearShot::Status HasClearShot::onUpdate() {
    if (((Vector2)robot->pos - Field::get_their_goal_center()).length() > Constants::MAX_SHOOTING_DISTANCE()) {
        return Status::Failure;
    }

    roboteam_msgs::World world = World::get_world();
    if (!control::ControlUtils::clearLine(ball->pos, Field::get_their_goal_center(), world, 1, false)) {
        return Status::Failure;
    }

    return Status::Success;
}

std::string HasClearShot::node_name() {return "HasClearShot";}

}
};
