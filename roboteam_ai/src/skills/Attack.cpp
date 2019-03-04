//
// Created by thijs on 17-12-18.
//

#include "Attack.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void Attack::onInitialize() {
    ownGoal = properties->getBool("ownGoal");
    goToPos.setAvoidBall(true);
}

// TODO: WTF HARDCODED SHIT EVERYWHERE
/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;
    Vector2 ball = World::getBall()->pos;
    Vector2 behindBall = Coach::getPositionBehindBallToGoal(0.35, ownGoal);
    Vector2 deltaBall = behindBall - ball;

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;

    GoToType goToType;

    if (!Coach::isRobotBehindBallToGoal(0.4, ownGoal, robot->pos)) {
        targetPos = behindBall;
        command.use_angle = 1;
        command.w = static_cast<float>((ball - (Vector2) (robot->pos)).angle());
        goToType = GoToType::clean;
        goToPos.setAvoidBall(true);

        // TODO: HACK HACK CHECK OPPONENT'S GOAL AND NOT OUR GOAL
        if (abs(((Vector2) robot->pos - targetPos).length()) < 0.1) {
            goToType = GoToType::basic;
            goToPos.setAvoidBall(false);
        }
    }
    else {
        targetPos = ball;
        goToPos.setAvoidBall(false);
        command.use_angle = 1;
        command.w = static_cast<float>(((Vector2) {- 1.0, - 1.0}*deltaBall).angle());
        // command.w = static_cast<float>((ball - robot->pos).angle());
        if (World::ourBotHasBall(robot->id, Constants::MAX_KICK_RANGE())) {
            command.kicker = 1;
            command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER());
            command.kicker_forced = 1;
        }
        goToType = GoToType::basic;
        goToPos.setAvoidBall(false);
    }
    Vector2 velocity;
    if (Field::pointIsInDefenceArea(robot->pos, ownGoal, 0.0)) {
        velocity = ((Vector2) robot->pos - Field::get_our_goal_center()).stretchToLength(2.0);
    }
    else if (Field::pointIsInDefenceArea(robot->pos, ownGoal, 0.0)) {
        velocity = ((Vector2) robot->pos - Field::get_their_goal_center()).stretchToLength(2.0);
    }
    else if (Field::pointIsInDefenceArea(ball, ownGoal) || Field::pointIsInDefenceArea(ball, ownGoal)) {
        velocity = {0, 0};
    }
    else if (Field::pointIsInDefenceArea(targetPos, ownGoal)) {
        velocity = {0, 0};
    }
    else {
        velocity = goToPos.goToPos(robot, targetPos, goToType);
    }

    if (velocity.length() < 0.6 && velocity.length() > 0.04) {
        velocity = velocity.stretchToLength(0.6);
    }

    velocity = control::ControlUtils::VelocityLimiter(velocity);

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