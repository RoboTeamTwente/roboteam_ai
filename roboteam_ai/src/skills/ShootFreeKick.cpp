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
    Vector2 goal = Field::get_their_goal_center();
    targetPos = ballPos + (robotPos - ballPos).rotate((goal - ballPos).angle());
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
                command.geneva_state = 1;
                Vector2 velocity = goToPos.goToPos(robot, ballPos, control::PosControlType::BASIC).vel;
                command.x_vel = static_cast<float>(velocity.x);
                command.y_vel = static_cast<float>(velocity.y);
                publishRobotCommand(command);

            }
            return Status::Running;
        }
        case TARGETING: {
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
}
}