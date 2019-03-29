//
// Created by robzelluf on 1/22/19.
//

#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include "Pass.h"

namespace rtt {
namespace ai {
Pass::Pass(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void Pass::onInitialize() {
    numTreeGtp.setAvoidBall(true);
    basicGtp.setAvoidBall(false);

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
                targetPos = coach::g_generalPositionCoach.getPositionBehindBallToPosition(0.30, robotToPassTo->pos);
                // use numtree
                control::PosVelAngle velocities = numTreeGtp.getPosVelAngle(robot, targetPos);
                command.x_vel = static_cast<float>(velocities.vel.x);
                command.y_vel = static_cast<float>(velocities.vel.y);

            } else if (!World::ourBotHasBall(robot->id)) {
                targetPos = ball->pos;
                // use basic
                control::PosVelAngle velocities = basicGtp.getPosVelAngle(robot, targetPos);
                command.x_vel = static_cast<float>(velocities.vel.x);
                command.y_vel = static_cast<float>(velocities.vel.y);
            } else {
                if (coach::g_pass.isReadyToReceivePass()) currentProgress = Progression::KICKING;
                return Status::Running;
            }
            command.w = static_cast<float>(((Vector2) robotToPassTo->pos - ball->pos).angle());
            command.dribbler = 0;
            break;
        }
        case Progression::KICKING: {
            if (World::ourBotHasBall(robot->id, Constants::MAX_KICK_RANGE())) {
                command.kicker = 1;
                command.kicker_forced = 1;
                distance = ((Vector2)ball->pos - robotToPassTo->pos).length();
                kicker_vel_multiplier = distance > rtt::ai::Constants::MAX_POWER_KICK_DISTANCE() ? 1.0 : distance / rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();
                command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER() * kicker_vel_multiplier);

                targetPos = ball->pos;
                Vector2 velocities = basicGtp.getPosVelAngle(robot, targetPos).vel;
                if (velocities.length() < 0.4) velocities = velocities.stretchToLength(0.4);

                command.x_vel = static_cast<float>(velocities.x);
                command.y_vel = static_cast<float>(velocities.y);
                command.w = static_cast<float>(((Vector2) robotToPassTo->pos - ball->pos).angle());

                publishRobotCommand();
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

    publishRobotCommand();
    return Status::Running;
}

}
}

