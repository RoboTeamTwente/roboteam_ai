//
// Created by baris on 15-1-19.
//

#include "BasicGoToPos.h"

namespace rtt {
namespace ai {

BasicGoToPos::BasicGoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void BasicGoToPos::onInitialize() {
    robot = getRobotFromProperties(properties);
    targetPos = properties->getVector2("target");
    if (properties->getBool("BallPlacementBefore")) {
        if (ball) {
            targetPos=coach::Coach::getBallPlacementBeforePos(ball->pos);
            targetPos = coach::Coach::getBallPlacementPos();
        }
        else {
            ROS_ERROR("BasicGoToPos: No ball found! assuming (%f,%f)", targetPos.x, targetPos.y);
        }
    }
    else if (properties->getBool("BallPlacementAfter")) {
        if (ball) {
            errorMargin = 0.05;
            targetPos = coach::Coach::getBallPlacementAfterPos(robot->angle);
        }
        else {
            ROS_ERROR("BasicGoToPos: No ball found! assuming (%f,%f)", targetPos.x, targetPos.y);
        }
    }
    goToPos.setAvoidBall(properties->getBool("avoidBall"));
    goToPos.setCanGoOutsideField(properties->getBool("canGoOutsideField"));
}

Skill::Status BasicGoToPos::onUpdate() {

    if (! robot) return Status::Running;

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>((targetPos - robot->pos).angle());
    if (properties->getBool("BallPlacementAfter")) {
        command.w = static_cast<float>((Vector2(robot->pos) - targetPos).angle());
    }
    Vector2 velocity = goToPos.goToPos(robot, targetPos, control::PosControlType::NUMERIC_TREES).vel;
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
