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
    amIClosest = false;
}

/// Called when the Skill is Updated
Pass::Status Pass::onUpdate() {
    if (! robot)
        return Status::Running;

    if ((Coach::getRobotClosestToBall(true)->id == robot->id) && ! amIClosest) {
        amIClosest = true;
        if (defensive) {
            robotToPass = coach::Coach::pickDefensivePassTarget(robot->id);
        }
        else {
            robotToPass = coach::Coach::pickOffensivePassTarget(robot->id, properties->getString("ROLE"));
        }
    }
    else if (! ((Coach::getRobotClosestToBall(true)->id == robot->id) && amIClosest)) {
        amIClosest = false;
        robotToPass = - 1;
    }

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    Vector2 velocity;

    if (robotToPass != - 1) {
        auto ball = World::getBall();
        Vector2 behindBall = Coach::getPositionBehindBallToRobot(0.5, true,
                static_cast<const unsigned int &>(robotToPass));
        Vector2 deltaBall = behindBall - ball->pos;

        GoToType goToType;

        if (! Coach::isRobotBehindBallToRobot(0.5, true, static_cast<const unsigned int &>(robotToPass), robot->pos)) {
            targetPos = behindBall;
            command.use_angle = 1;
            command.w = static_cast<float>(((Vector2) (ball->pos) - ((Vector2) robot->pos)).angle());
            goToType = GoToType::luTh;
            if (abs(((Vector2) robot->pos - targetPos).length()) < 1.0) goToType = GoToType::basic;
        }
        else {
            targetPos = ball->pos;
            command.use_angle = 1;
            command.w = static_cast<float>((deltaBall.stretchToLength(- 1.0)).angle());
            if (Coach::doesRobotHaveBall(robot->id, true) && ((Vector2) (ball->vel)).length() < 1.0f) {
                command.kicker = 1;
                double ballDistance = ((Vector2) ball->pos - robotToPass).length();
                auto kickVel = static_cast<float> (ballDistance > 6.0f ?
                                                   rtt::ai::constants::MAX_KICK_POWER :
                                                   rtt::ai::constants::MAX_KICK_POWER*ballDistance/9.0f+3.0f );

                command.kicker_vel = kickVel;
                command.kicker_forced = 1;
            }
            goToType = GoToType::basic;
        }

        if (Field::pointIsInDefenceArea(robot->pos, true, 0.2))
            velocity = ((Vector2) robot->pos - Field::get_our_goal_center()).stretchToLength(2.0);
        else if (Field::pointIsInDefenceArea(robot->pos, false, 0.2))
            velocity = ((Vector2) robot->pos - Field::get_their_goal_center()).stretchToLength(2.0);
        else if (Field::pointIsInDefenceArea(ball->pos, true) || Field::pointIsInDefenceArea(ball->pos, false))
            velocity = {0, 0};
        else
            velocity = goToPos.goToPos(robot, targetPos, goToType);
    }
    else {
        if (((Vector2) (ball->vel)).length() < 0.5f) {
            Vector2 otherRobot = Coach::getRobotClosestToBall(true)->pos;
            targetPos = otherRobot - (otherRobot-robot->pos).stretchToLength(1.0f);
            velocity = goToPos.goToPos(robot, targetPos, GoToType::luTh);
        }
        else {
            Vector2 a1 = ball->pos;
            Vector2 a2 = (Vector2) (ball->pos) + (Vector2) (ball->vel);
            Vector2 b1 = robot->pos;
            Vector2 b2 = (Vector2) robot->pos + (Vector2) {a2.y, a2.x};
            targetPos = control::ControlUtils::twoLineIntersection(a1, a2, b1, b2);
            velocity = goToPos.goToPos(robot, targetPos, GoToType::basic);
        }
        command.use_angle = 1;
        command.w = static_cast<float>(((Vector2) (ball->pos) - (Vector2) (robot->pos)).angle());

    }

    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand(command);

    return Status::Running;
}

} // ai
} // rtt
