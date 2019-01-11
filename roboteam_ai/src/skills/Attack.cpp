//
// Created by thijs on 17-12-18.
//

#include "Attack.h"
#include "../utilities/Coach.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

/// Init the GoToPos skill
void Attack::onInitialize() {
    robot = getRobotFromProperties(properties);
}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;
    Vector2 ballPos = ball->pos;
    Vector2 behindBall = coach::Coach::getPositionBehindBall(0.5);
    Vector2 deltaBall = behindBall - ballPos;
    if (! Control::pointInTriangle(robot->pos, ballPos, ballPos + (deltaBall).rotate(M_PI*0.17).scale(2.0),
            ballPos + (deltaBall).rotate(M_PI*- 0.17).scale(2.0))) {
        targetPos = behindBall;

        roboteam_msgs::RobotCommand command;
        command.id = robot->id;
        command.use_angle = 1;
        command.w = static_cast<float>((ballPos - (Vector2) (robot->pos)).angle());
        Vector2 velocity = goToPos.goToPos(robot, targetPos, control::GoToType::basic);
        command.x_vel = static_cast<float>(velocity.x);
        command.y_vel = static_cast<float>(velocity.y);
        publishRobotCommand(command);
    }
    else {

        targetPos = ballPos;

        roboteam_msgs::RobotCommand command;
        command.id = robot->id;
        command.use_angle = 1;
        command.w = static_cast<float>(((Vector2) {- 1.0, - 1.0}*deltaBall).angle());
        if (coach::Coach::doesRobotHaveBall(robot->id, true)) {
            command.kicker = 1;
            command.kicker_vel = static_cast<float>(rtt::ai::constants::MAX_KICK_POWER);
            command.kicker_forced = 1;
        }
        Vector2 velocity = goToPos.goToPos(robot, targetPos, control::GoToType::basic);
        command.x_vel = static_cast<float>(velocity.x);
        command.y_vel = static_cast<float>(velocity.y);
        publishRobotCommand(command);
    }

    return Status::Running;
}

void Attack::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(deltaPos.angle());

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

} // ai
} // rtt