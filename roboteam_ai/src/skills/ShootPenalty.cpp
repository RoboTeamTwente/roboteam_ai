//
// Created by baris on 11-3-19.
//

#include "ShootPenalty.h"

namespace rtt {
namespace ai {

void ShootPenalty::onInitialize() {
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 robotPos = robot->pos;
    targetPos = ballPos + (robotPos - ballPos).rotate(fakeOffset.getAngle());
    progress = GOING;
}

Skill::Status ShootPenalty::onUpdate() {

    switch (progress) {

        case GOING: {
            Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
            Vector2 deltaPos = (ballPos - robot->pos);

            if (deltaPos.length() < errorMarginPos) {
                progress = ROTATING;
            } else {
                command.w = static_cast<float>((ballPos - robot->pos).angle());
                command.geneva_state = 1;
                Vector2 velocity = goToPos.getPosVelAngle(robot, ballPos).vel;
                command.x_vel = static_cast<float>(velocity.x);
                command.y_vel = static_cast<float>(velocity.y);
                publishRobotCommand();

            }
            return Status::Running;
        }

        case ROTATING: {
            if ((robot->angularVelocity - fakeOffset) > errorMarginAng) {
                command.geneva_state = 1;
                command.w = static_cast<float>(fakeOffset);
                publishRobotCommand();

            }
            else {
                progress = READY;
                return Status::Running;
            }

            return Status::Running;
        }

        case READY: {
            publishRobotCommand();
            progress = SHOOTING;
            return Status::Running;
        }

        case SHOOTING: {
            if (! isPenaltyShot()) {
                Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
                command.w = static_cast<float>((ballPos - robot->pos).angle());
                command.geneva_state = 1;
                command.kicker = static_cast<unsigned char>(true);
                command.kicker_vel = Constants::MAX_KICK_POWER();
                Vector2 velocity = goToPos.getPosVelAngle(robot, ballPos).vel;
                command.x_vel = static_cast<float>(velocity.x);
                command.y_vel = static_cast<float>(velocity.y);
                publishRobotCommand();
                return Status::Running;
            }
            else {
                publishRobotCommand();
                return Status::Success;
            }
        }

    }

    return Status::Failure;
}

void ShootPenalty::onTerminate(Skill::Status s) {
    // clean up the coach or whereever logic you use

}
ShootPenalty::ShootPenalty(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
bool ShootPenalty::isPenaltyShot() {
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    return ((ballPos - rtt::ai::world::field->getPenaltyPoint(false)).length() > 0.05);

}

}
}