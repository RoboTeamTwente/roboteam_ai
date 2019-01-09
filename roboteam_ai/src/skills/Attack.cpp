//
// Created by thijs on 17-12-18.
//

#include "Attack.h"
#include "../utilities/Coach.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) { }

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (!robot || !ball) return Status::Running;

    Vector2 ballPos = ball->pos;
    Vector2 behindBall = coach::Coach::getPositionBehindBall(0.5);

    Vector2 deltaBall = behindBall - ballPos;
    if (! Control::pointInTriangle(robot->pos, ballPos-deltaBall, ballPos + (deltaBall).rotate(M_PI*0.17).scale(2.0),
            ballPos + (deltaBall).rotate(M_PI*- 0.17).scale(2.0))) {
        targetPos = behindBall;
        goToPos.goToPos(robot, targetPos, control::GoToType::luTh);
        std::cout << "luth\n";
    }
    else {
        roboteam_msgs::RobotCommand command;
        command.id = robot->id;
        command.use_angle = 1;
        command.w = static_cast<float>((ballPos - behindBall).angle());
        publishRobotCommand(command);
        targetPos = ballPos;
        goToPos.goToPos(robot, targetPos, control::GoToType::basic);

        if (coach::Coach::doesRobotHaveBall(robot->id, true)) {
            unsigned char forced_kick = 1;
            kicker.kick(robot, forced_kick);
        }
        std::cout << "basic\n";

    }

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