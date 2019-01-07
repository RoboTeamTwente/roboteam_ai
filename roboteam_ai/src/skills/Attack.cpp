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

    if (!Control::pointInTriangle(robot->pos, ball, ball + (behindBall-ball).rotate(M_PI*0.17).scale(1.74),
            ball + (behindBall-ball).rotate(M_PI*-0.17).scale(1.74))) {
        targetPos = behindBall;
        goToPos.goToPos(robot, targetPos, GoToType::luTh);
    } else if (!Coach::doesRobotHaveBall(robot->id, true)) {
        targetPos = ball;
        goToPos.goToPos(robot, targetPos, GoToType::basic);
    } else {
        targetPos = ball;
        unsigned char forced_kick = 1;
        kicker.kick(robot, forced_kick);
        goToPos.goToPos(robot, targetPos, GoToType::basic);
        roboteam_msgs::RobotCommand command;
        command.id = robot->id;
        command.use_angle = 1;
        command.w = static_cast<float>((ball-behindBall).angle());
        publishRobotCommand(command);
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