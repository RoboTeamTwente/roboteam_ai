//
// Created by baris on 21-2-19.
//

#include "GoBehindBall.h"

namespace rtt {
namespace ai {

GoBehindBall::GoBehindBall(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {

}

Skill::Status GoBehindBall::onUpdate() {

    auto ball = ai::World::getBall();
    auto goal = ai::Field::get_their_goal_center();

    Vector2 v = goal - ball->pos;
    auto targetPos = ((v * -1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS())) + ball->pos;
    // TODO draw the point in the interface

    Vector2 velocity = goToPos.goToPos(robot, targetPos, control::GoToType::clean);

    Vector2 deltaPos = targetPos - robot->pos;
    publishCommand(targetPos, velocity);


    if (deltaPos.length() > errorMargin) {
        return Status::Running;
    }
    else {
        return Status::Success;
    }
}

void GoBehindBall::onInitialize() {
    if (properties->hasString("type")) {
        type = stringToUnit(properties->getString("type"));
    }
}

void GoBehindBall::onTerminate(Skill::Status s) {
    std::cout << "terminate" << std::endl;

}

GoBehindBall::unit GoBehindBall::stringToUnit(std::string string) {
    if (string == "penalty") {
        return penalty;
    }
    else if (string == "corner") {
        return corner;
    }
    else {
        return freeKick;
    }
}
void GoBehindBall::publishCommand(Vector2 targetPos , Vector2 velocity) {

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>((targetPos-robot->pos).angle());
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand(command);

}
}
}