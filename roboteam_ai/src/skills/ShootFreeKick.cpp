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
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 robotPos = robot->pos;
    freeKickPos = ballPos;
    Vector2 goal = world::field->get_their_goal_center();
    // behind the ball looking at the goal
    targetPos = ballPos + (ballPos - goal).stretchToLength(Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS() + 0.03);
    progress = GOING;
}

Skill::Status ShootFreeKick::onUpdate() {

    switch (progress) {

        case GOING: {
            Vector2 deltaPos = (targetPos - robot->pos);

            if (deltaPos.length() < errorMarginPos) {
                progress = TARGETING;
            }
            else {
                roboteam_msgs::RobotCommand command;
                command.id = robot->id;
                command.use_angle = 1;
                command.w = static_cast<float>((targetPos - robot->pos).angle());
                Vector2 velocity = goToPos.getPosVelAngle(robot, targetPos).vel;
                command.x_vel = static_cast<float>(velocity.x);
                command.y_vel = static_cast<float>(velocity.y);
                publishRobotCommand(command);

            }
            return Status::Running;
        }

        case TARGETING: {
            Vector2 deltaPos = (targetPos - robot->pos);

            if (deltaPos.length() < errorMarginPos) {
                progress = READY;
            }
            else {
                // Find a target and draw a vector to it
                // TODO make targeting functions based on our robots positions maybe coach
                // TODO for now it shoots at the goal
                Vector2 target = rtt::ai::world::field->getPenaltyPoint(false);
                Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
                targetPos = ballPos + (ballPos - target).stretchToLength(
                        Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS() + 0.03);

            }
            return Status::Running;

        }

        case READY: {
            progress = SHOOTING;
            return Status::Running;
            // TODO inform the coach that we are ready to take the free kick and do some other comm.
        }

        case SHOOTING: {
            if (! isShot()) {
                Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
                roboteam_msgs::RobotCommand command;
                command.id = robot->id;
                command.use_angle = 1;
                command.w = static_cast<float>((ballPos - robot->pos).angle());
                command.chipper = static_cast<unsigned char>(true);
                command.chipper_vel = 8.0; // Such power much wow
                Vector2 velocity = goToPos.getPosVelAngle(robot, ballPos).vel;
                command.x_vel = static_cast<float>(velocity.x);
                command.y_vel = static_cast<float>(velocity.y);
                publishRobotCommand(command);
                return Status::Running;
            }
            else {
                // Defaults to zero
                roboteam_msgs::RobotCommand command;
                command.id = robot->id;
                command.use_angle = 1;
                publishRobotCommand(command);
                return Status::Success;
            }        }
    }

    return Status::Waiting;
}

void ShootFreeKick::onTerminate(Skill::Status s) {
}

bool ShootFreeKick::isShot() {
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    return ((ballPos - freeKickPos).length() > 0.05);

}
}
}