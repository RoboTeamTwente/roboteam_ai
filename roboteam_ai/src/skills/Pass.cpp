//
// Created by robzelluf on 1/22/19.
//

#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include <roboteam_ai/src/utilities/Constants.h>
#include "Pass.h"

namespace rtt {
namespace ai {
Pass::Pass(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void Pass::onInitialize() {
    goToPos.setAvoidBall(true);
    robotToPassToID = coach::g_pass.initiatePass();
    currentProgress = Progression::POSITIONING;
}

Pass::Status Pass::onUpdate() {
    if (robotToPassToID == -1) return Status::Failure;
    robotToPassTo = World::getRobotForId(static_cast<unsigned int>(robotToPassToID), true);

    roboteam_msgs::RobotCommand command;

    switch(currentProgress) {
        case Progression::POSITIONING: {
            if (!coach::g_generalPositionCoach.isRobotBehindBallToPosition(0.30, robotToPassTo->pos, robot->pos)) {
                goToType = GoToType::NUMERIC_TREES;
                targetPos = coach::g_generalPositionCoach.getPositionBehindBallToPosition(0.30, robotToPassTo->pos);
            } else if (!World::ourBotHasBall(robot->id)) {
                goToType = GoToType::BASIC;
                targetPos = ball->pos;
                goToPos.setAvoidBall(false);
            } else {
                if (coach::g_pass.isReadyToReceivePass()) currentProgress = Progression::KICKING;
                return Status::Running;
            }
            command.use_angle = 1;
            command.w = static_cast<float>(((Vector2) robotToPassTo->pos - ball->pos).angle());
            command.dribbler = 0;
            control::PosVelAngle velocities = goToPos.goToPos(robot, targetPos, goToType);
            command.x_vel = static_cast<float>(velocities.vel.x);
            command.y_vel = static_cast<float>(velocities.vel.y);
            break;
        }
        case Progression::KICKING: {
            if (World::ourBotHasBall(robot->id, Constants::MAX_KICK_RANGE())) {
                command.kicker = 1;
                command.kicker_forced = 1;
                distance = ((Vector2)ball->pos - robotToPassTo->pos).length();
                kicker_vel_multiplier = distance > rtt::ai::Constants::MAX_POWER_KICK_DISTANCE() ? 1.0 : distance / rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();
                command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER() * kicker_vel_multiplier);
                command.id = robot->id;

                goToType = GoToType::BASIC;
                targetPos = ball->pos;
                Vector2 velocities = goToPos.goToPos(robot, targetPos, goToType).vel;
                if (velocities.length() < 0.4) velocities = velocities.stretchToLength(0.4);

                command.x_vel = static_cast<float>(velocities.x);
                command.y_vel = static_cast<float>(velocities.y);
                command.w = static_cast<float>(((Vector2) robotToPassTo->pos - ball->pos).angle());

                publishRobotCommand(command);
                checkTicks = 0;
                return Status::Running;
            }

            if (Vector2(ball->vel).length() > 0.4 || ((Vector2)robot->pos - ball->pos).length() > rtt::ai::Constants::MAX_BALL_RANGE() * 2) {
                coach::g_pass.setRobotBeingPassedTo(-1);
                coach::g_pass.setPassed(true);
                return Status::Success;
            } else if (checkTicks < maxCheckTicks) {
                checkTicks++;
            } else {
                currentProgress = Progression::POSITIONING;
                checkTicks = 0;
                return Status::Running;
            }
        }
    }

    command.id = robot->id;
    publishRobotCommand(command);
    return Status::Running;
}

}
}

