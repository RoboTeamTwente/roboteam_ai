//
// Created by baris on 14-3-19.
//

#include "skills/ShootFreeKick.h"
#include <world/World.h>

namespace rtt::ai {

ShootFreeKick::ShootFreeKick(string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {}

void ShootFreeKick::onInitialize() {
    Vector2 ballPos = world->getBall()->getPos();
    freeKickPos = ballPos;
    Vector2 goal = (*field).getTheirGoalCenter();

    // behind the ball looking at the goal
    targetPos = ballPos + (ballPos - goal).stretchToLength(Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS() + 0.03);
    progress = GOING;
}

Skill::Status ShootFreeKick::onUpdate() {
    Vector2 target;
    switch (progress) {
        case GOING: {
            Vector2 deltaPos = (targetPos - robot->pos);

            if (deltaPos.length() < errorMarginPos) {
                progress = TARGETING;
            } else {
                command.set_w(static_cast<float>((targetPos - robot->pos).angle()));
                Vector2 velocity = goToPos.getRobotCommand(world, field, robot, targetPos).vel;
                command.mutable_vel()->set_x(static_cast<float>(velocity.x));
                command.mutable_vel()->set_y(static_cast<float>(velocity.y));
                publishRobotCommand();
            }
            return Status::Running;
        }

        case TARGETING: {
            Vector2 deltaPos = (targetPos - robot->pos);

            if (deltaPos.length() < errorMarginPos) {
                progress = READY;
            } else {
                // Find a target and draw a vector to it
                // TODO make targeting functions based on our robots positions maybe coach
                // TODO for now it shoots at the goal
                Vector2 target = FieldComputations::getPenaltyPoint(*field, false);
                Vector2 ballPos = world->getBall()->getPos();
                targetPos = ballPos + (ballPos - target).stretchToLength(Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS() + 0.03);
            }
            return Status::Running;
        }

        case READY: {
            if (counter < 90) {
                counter++;
                return Status::Running;
            }
            progress = SHOOTING;
            return Status::Running;
            // TODO inform the coach that we are ready to take the free kick and do some other comm.
        }

        case SHOOTING: {
            if (!isShot()) {
                Vector2 target = FieldComputations::getPenaltyPoint(*field, false);

                auto shotData = robot->getShotController()->getRobotCommand(*field, *robot, target, false, control::BallSpeed::PASS);
                command = shotData.makeROSCommand();
                publishRobotCommand();
                return Status::Running;
            } else {
                // Defaults to zero
                publishRobotCommand();
                return Status::Success;
            }
        }
    }

    return Status::Waiting;
}

void ShootFreeKick::onTerminate(Skill::Status s) {
    counter = 0;
    progress = GOING;
}

bool ShootFreeKick::isShot() {
    Vector2 ballPos = world->getBall()->getPos();
    return ((ballPos - freeKickPos).length() > 0.05);
}
}  // namespace rtt::ai