//
// Created by thijs on 17-12-18.
//

#include "Attack.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) { }

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (!robot || !ball) return Status::Running;

    Vector2 ballPos = ball->pos;
    Vector2 behindBall = Coach::getPositionBehindBall(0.5);

    if (!Control::pointInTriangle(robot->pos, ballPos, ballPos + (behindBall-ballPos).rotate(M_PI*0.3).scale(1.5),
            ballPos + (behindBall-ballPos).rotate(M_PI*-0.3).scale(1.5))) {
        targetPos = behindBall;
    } else if (!Coach::doesRobotHaveBall(robot->id, true)) {
        targetPos = ballPos;
    } else {
        targetPos = ballPos;
        unsigned char forced_kick = 1;
        kicker.kick(robot, forced_kick);
    }
    goToPos.goToPos(robot, targetPos, GoToType::luTh);


    return Status::Running;
}

void Attack::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(deltaPos.angle());

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

} // ai
} // rtt