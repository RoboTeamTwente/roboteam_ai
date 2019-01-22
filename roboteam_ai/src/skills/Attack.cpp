//
// Created by thijs on 17-12-18.
//

#include "Attack.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;
    Vector2 ball = World::getBall()->pos;
    Vector2 behindBall = Coach::getPositionBehindBallToGoal(0.5, true);
    Vector2 deltaBall = behindBall - ball;

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;


    GoToType goToType;

    if (! Coach::isRobotBehindBallToGoal(0.5, true, robot->pos)) {
        targetPos = behindBall;
        command.use_angle = 1;
        command.w = static_cast<float>((ball - (Vector2) (robot->pos)).angle());
        goToType = GoToType::luTh;
        if (abs(((Vector2) robot->pos - targetPos).length()) < 1.0) goToType = GoToType::basic;
    }
    else {
        targetPos = ball;
        command.use_angle = 1;
        command.w = static_cast<float>(((Vector2) {- 1.0, - 1.0}*deltaBall).angle());
        if (Coach::doesRobotHaveBall(robot->id, true, 0.05, 0.1)) {
            command.kicker = 1;
            command.kicker_vel = static_cast<float>(rtt::ai::constants::MAX_KICK_POWER);
            command.kicker_forced = 1;
        }
        goToType = GoToType::basic;
    }
    Vector2 velocity;
    if (Field::pointIsInDefenceArea(robot->pos, true, 0.2)) {
        velocity = ((Vector2) robot->pos - Field::get_our_goal_center()).stretchToLength(2.0);
    }
    else if (Field::pointIsInDefenceArea(robot->pos, false, 0.2)) {
        velocity = ((Vector2) robot->pos - Field::get_their_goal_center()).stretchToLength(2.0);

    } else if (Field::pointIsInDefenceArea(ball, true) || Field::pointIsInDefenceArea(ball, false)) {
        velocity = {0, 0};
    } else {
        velocity = goToPos.goToPos(robot, targetPos, goToType);
    }
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand(command);

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