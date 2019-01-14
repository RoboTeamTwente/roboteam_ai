//
// Created by baris on 5-12-18.
//

#include "Pass.h"
namespace rtt {
namespace ai {

Pass::Pass(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

/// Called when the Skill is Initialized
void Pass::onInitialize() {
    defensive = properties->getBool("defensive");
    robotToPass = - 1;
    newTarget = false;
}

/// Called when the Skill is Updated
Pass::Status Pass::onUpdate() {
    bool amIclosest = true;

    if (newTarget && amIclosest) {
        robotToPass = -1;
        newTarget = false;
    }

    if (robotToPass == - 1) {
        if (defensive) {
            robotToPass = Coach::pickDefensivePassTarget(robot->id);
        }
        else {
            robotToPass = Coach::pickOffensivePassTarget(robot->id, properties->getString("ROLE"));
        }
        return Status::Running;
    }
    if (! robot) return Status::Running;
    Vector2 ball = World::getBall()->pos;
    Vector2 behindBall = Coach::getPositionBehindBallToRobot(0.5, true, static_cast<const unsigned int &>(robotToPass));
    Vector2 deltaBall = behindBall - ball;

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;

    GoToType goToType;

    if (! Control::pointInTriangle(robot->pos, ball, ball + (deltaBall).rotate(M_PI*0.17).scale(2.0),
            ball + (deltaBall).rotate(M_PI*- 0.17).scale(2.0))) {
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
        if (Coach::doesRobotHaveBall(robot->id, true)) {
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

} // ai
} // rtt
