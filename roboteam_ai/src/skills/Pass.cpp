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
robotToPassToID = Coach::getRobotClosestToGoal(true, false);
robotToPassTo = World::getRobotForId(static_cast<unsigned int>(robotToPassToID), true);
currentProgress = INITIATING;
}

Pass::Status Pass::onUpdate() {
    if (robotToPassToID == -1) return Status::Failure;

    roboteam_msgs::RobotCommand command;

    switch(currentProgress) {
        case INITIATING: {
            if (coach::Coach::initiatePass(robotToPassToID)) {
                currentProgress = POSITIONING;
                return Status::Running;
            } else return Status::Failure;
        }
        case POSITIONING: {
            if (!coach::Coach::isRobotBehindBallToPosition(0.7, robotToPassTo->pos, robot->pos)) {
                goToType = GoToType::luTh;
                targetPos = Coach::getPositionBehindBallToPosition(0.7, robotToPassTo->pos);
            } else if (!coach::Coach::doesRobotHaveBall(robot->id, true, 0.15)) {
                goToType = GoToType::basic;
                targetPos = ball->pos;
            } else {
                currentProgress = KICKING;
                return Status::Running;
            }
            command.use_angle = 1;
            command.w = static_cast<float>(((Vector2) robotToPassTo->pos - ball->pos).angle());
            Vector2 velocities = goToPos.goToPos(robot, targetPos, goToType);
            command.x_vel = static_cast<float>(velocities.x);
            command.y_vel = static_cast<float>(velocities.y);
        }
        case KICKING: {
            if (true) {
                command.kicker = 1;
                command.kicker_forced = 1;
                command.kicker_vel = static_cast<float>(rtt::ai::constants::MAX_KICK_POWER);
                command.id = robot->id;
                publishRobotCommand(command);
                Coach::setRobotBeingPassedTo(-1);

                return Status::Success;
            }
        }
    }

    command.id = robot->id;
    publishRobotCommand(command);

    return Status::Running;
}

}
}

