//
// Created by thijs on 17-12-18.
//

#include "Attack.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Init the GoToPos skill
void Attack::onInitialize() {
    robot = getRobotFromProperties(properties);
}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    updateRobot();
    if (! robot) return Status::Running;
    Vector2 ball = World::getBall().pos;
    Vector2 behindBall = Coach::getPositionBehindBall(0.5);
    Vector2 deltaBall = behindBall - ball;
    if (! Control::pointInTriangle(robot->pos, ball-deltaBall, ball + (deltaBall).rotate(M_PI*0.17).scale(2.0),
            ball + (deltaBall).rotate(M_PI*- 0.17).scale(2.0))) {
        targetPos = behindBall;
        goToPos.goToPos(robot, targetPos, GoToType::luTh);
        std::cout << "luth\n";
    }
    else {
        roboteam_msgs::RobotCommand command;
        command.id = robot->id;
        command.use_angle = 1;
        command.w = static_cast<float>((ball - behindBall).angle());
        publishRobotCommand(command);
        targetPos = ball;
        goToPos.goToPos(robot, targetPos, GoToType::basic);

        if (Coach::doesRobotHaveBall(robot->id, true)) {
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