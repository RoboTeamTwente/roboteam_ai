//
// Created by baris on 15-1-19.
//

#include "BasicGoToPos.h"

namespace rtt {
namespace ai {

BasicGoToPos::BasicGoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {

}

void BasicGoToPos::onInitialize() {
    robot = getRobotFromProperties(properties);
    if (properties->hasBool("goToBall")) {
        if (properties->getBool("goToBall")) targetPos = ball->pos;
    }

    if (properties->hasVector2("target")) targetPos = properties->getVector2("target");
}

Skill::Status BasicGoToPos::onUpdate() {

    if (! robot) return Status::Running;

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>((targetPos-robot->pos).angle());
    Vector2 velocity = goToPos.goToPos(robot, targetPos, control::GoToType::luTh);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand(command);

    Vector2 deltaPos = targetPos - robot->pos;

    if (deltaPos.length() > errorMargin) {
        return Status::Running;
    }
    else {
        return Status::Success;
    }
}

}
}
