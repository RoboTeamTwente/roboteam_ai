//
// Created by robzelluf on 1/22/19.
//

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
    command.id = robot->id;

    switch(currentProgress) {
        case Progression::POSITIONING: {
            goToType = GoToType::NUMERIC_TREES;
            passPosition = robotToPassTo->pos;

            /// Check if robot is not yet at the targetPos
            if (!coach::g_generalPositionCoach.isRobotBehindBallToPosition(0.25, passPosition, robot->pos)) {
                targetPos = coach::g_generalPositionCoach.getPositionBehindBallToPosition(0.20, passPosition);
                std::cout << "Getting behind ball" << std::endl;
            } else if (!World::ourBotHasBall(robot->id, 0.15)) {
                targetPos = ball->pos;
                std::cout << "Getting towards ball" << std::endl;
            }
            /// Check if the robot does not have the ball
            else if (coach::g_pass.isReadyToReceivePass()) {
                currentProgress = Progression::KICKING;
                return Status::Running;
            }
            command.use_angle = 1;

            // TODO: Check if angle can be same as for the attacker that shoots at the goal
            command.w = static_cast<float>(((Vector2) robotToPassTo->pos - ball->pos).angle());
            command.dribbler = 0;
            Vector2 velocities = goToPos.goToPos(robot, targetPos, goToType).vel;
            velocities = control::ControlUtils::VelocityLimiter(velocities);
            command.x_vel = static_cast<float>(velocities.x);
            command.y_vel = static_cast<float>(velocities.y);
            break;
        }
        case Progression::KICKING: {
            std::cout << "Kicking" << std::endl;
            if (World::ourBotHasBall(robot->id, Constants::MAX_KICK_RANGE())) {
                command.kicker = 1;
                command.kicker_forced = 1;
                distance = ((Vector2)ball->pos - robotToPassTo->pos).length();
                kicker_vel_multiplier = distance > rtt::ai::Constants::MAX_POWER_KICK_DISTANCE() ? 1.0 : distance / rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();
                command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER() * kicker_vel_multiplier);

                goToType = GoToType::BASIC;
                targetPos = ball->pos;
                Vector2 velocities = goToPos.goToPos(robot, targetPos, goToType).vel;
                velocities = control::ControlUtils::VelocityLimiter(velocities);
                if (velocities.length() < 0.6) velocities = velocities.stretchToLength(0.6);

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
    publishRobotCommand(command);
    return Status::Running;
}

} // ai
} // rtt

