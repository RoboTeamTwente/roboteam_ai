//
// Created by baris on 14-3-19.
//

#include "ShootFreeKick.h"

namespace rtt {
namespace ai {

ShootFreeKick::ShootFreeKick(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}

void ShootFreeKick::onInitialize() {
    Vector2 ballPos = rtt::ai::World::getBall()->pos;
    Vector2 robotPos = robot->pos;
    freeKickPos = ballPos;
    Vector2 goal = Field::get_their_goal_center();
    // behind the ball looking at the goal
    targetPos = ballPos + (ballPos - goal).stretchToLength(Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS() + 0.03);
    progress = GOING;
}

Skill::Status ShootFreeKick::onUpdate() {

    switch (progress) {

        case GOING: {
            Vector2 ballPos = rtt::ai::World::getBall()->pos;
            Vector2 deltaPos = (ballPos - robot->pos);

            if (deltaPos.length() < errorMarginPos) {
                progress = TARGETING;
            }
            else {
                roboteam_msgs::RobotCommand command;
                command.id = robot->id;
                command.use_angle = 1;
                command.w = static_cast<float>((ballPos - robot->pos).angle());
                Vector2 velocity = goToPos.goToPos(robot, ballPos, control::PosControlType::BASIC).vel;
                command.x_vel = static_cast<float>(velocity.x);
                command.y_vel = static_cast<float>(velocity.y);
                publishRobotCommand(command);

            }
            return Status::Running;
        }
        case TARGETING: {
            // Find a target and draw a vector to it
            Vector2
            break;
        }
        case READY: {
            break;
        }
        case SHOOTING: {
            break;
        }
    }

    return Status::Waiting;
}

void ShootFreeKick::onTerminate(Skill::Status s) {
}

bool ShootFreeKick::isShot() {
    Vector2 ballPos = rtt::ai::World::getBall()->pos;
    return ((ballPos - freeKickPos).length() > 0.05);

}
}
}