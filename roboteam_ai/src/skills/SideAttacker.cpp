//
// Created by thijs on 17-12-18.
//

#include "SideAttacker.h"

namespace rtt {
namespace ai {

SideAttacker::SideAttacker(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void SideAttacker::onInitialize() {
    targetPos = coach::g_offensiveCoach.calculatePositionForRobot(robot);
}


/// Get an update on the skill
bt::Node::Status SideAttacker::onUpdate() {
    if (! robot) return Status::Running;
    targetPos = coach::g_offensiveCoach.calculatePositionForRobot(robot);

    auto newPosition = goToPos.getPosVelAngle(robot, targetPos);
    Vector2 velocity = newPosition.vel;
    velocity = control::ControlUtils::velocityLimiter(velocity);

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    command.w = static_cast<float>(newPosition.angle);
    publishRobotCommand(command);
    return Status::Running;
}

void SideAttacker::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(deltaPos.angle());

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
    coach::g_offensiveCoach.releaseRobot(robot->id);
}

} // ai
} // rtt