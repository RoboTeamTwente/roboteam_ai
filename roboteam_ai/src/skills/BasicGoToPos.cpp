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
    targetPos = properties->getVector2("target");
}

Skill::Status BasicGoToPos::onUpdate() {

    if (! robot) return Status::Running;

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    Vector2 velocity = goToPos.goToPos(robot, targetPos, control::GoToType::luTh);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    command.w = 0.0f;//static_cast<float>((targetPos-robot->pos).length());
    publishRobotCommand(command);

    double dx = targetPos.x - robot->pos.x;
    double dy = targetPos.y - robot->pos.y;
    Vector2 deltaPos = {dx, dy};

    if (deltaPos.length() > errorMargin) {
        return Status::Running;
    }
    else {
        return Status::Success;
    }
}

}
}
