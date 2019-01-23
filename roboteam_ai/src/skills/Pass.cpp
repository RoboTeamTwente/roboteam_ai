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
robotToPassToID = robotDealer::RobotDealer::findRobotForRole("receiver");
robotToPassTo = World::getRobotForId(static_cast<unsigned int>(robotToPassToID), true);
currentProgress = Progression::INITIATING;
std::cout << "NEW PASS" << std::endl;
}

Pass::Status Pass::onUpdate() {
    if (robotToPassToID == -1) return Status::Failure;

    roboteam_msgs::RobotCommand command;

    switch(currentProgress) {
        case Progression::INITIATING:
            if (coach::Coach::initiatePass(robotToPassToID)) {
                currentProgress = Progression::POSITIONING;
                return Status::Running;
            } else return Status::Failure;
        case Progression::POSITIONING: {
            std::cout << "POSITIONING" << std::endl;
            if (!coach::Coach::isRobotBehindBallToPosition(0.15, robotToPassTo->pos, robot->pos)) {
                goToType = GoToType::luTh;
                targetPos = Coach::getPositionBehindBallToPosition(0.15, robotToPassTo->pos);
            } else if (!coach::Coach::doesRobotHaveBall(robot->id, true, rtt::ai::constants::MAX_BALL_RANGE)) {
                goToType = GoToType::basic;
                targetPos = ball->pos;
            } else {
                currentProgress = Progression::KICKING;
                return Status::Running;
            }
            command.use_angle = 1;
            command.w = static_cast<float>(((Vector2) robotToPassTo->pos - ball->pos).angle());
            Vector2 velocities = goToPos.goToPos(robot, targetPos, goToType);
            command.x_vel = static_cast<float>(velocities.x);
            command.y_vel = static_cast<float>(velocities.y);
            break;
        }
        case Progression::KICKING: {
            // TODO: check whether team mate is ready to receive pass
            if (coach::Coach::doesRobotHaveBall(robot->id, true, rtt::ai::constants::MAX_BALL_RANGE)) {
                command.kicker = 1;
                command.kicker_forced = 1;
                distance = ((Vector2)ball->pos - robotToPassTo->pos).length();
                kicker_vel_multiplier = distance > maxDistance ? 1.0 : distance / maxDistance;

                command.kicker_vel = static_cast<float>(rtt::ai::constants::MAX_KICK_POWER * kicker_vel_multiplier);
                command.id = robot->id;
                std::cout << "KICK" << std::endl;
                publishRobotCommand(command);
                checkTicks = 0;
                return Status::Running;
            }
            if (Vector2(ball->vel).length() > 0.4 && ((Vector2)robot->pos - ball->pos).length() > rtt::ai::constants::MAX_BALL_RANGE * 3) {
                Coach::setRobotBeingPassedTo(-1);
                Coach::setPassed(true);
                return Status::Success;
            } else if (checkTicks < maxCheckTicks) {
                std::cout << Vector2(ball->vel).length() << std::endl;
                checkTicks++;
                std::cout << checkTicks << " - " << maxCheckTicks << std::endl;
                break;
            };
            currentProgress = Progression::POSITIONING;
            return Status::Running;
        }
    }

    command.id = robot->id;
    publishRobotCommand(command);
    return Status::Running;
}

}
}

