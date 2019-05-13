//
// Created by thijs on 17-12-18.
//

#include "SideAttacker.h"

namespace rtt {
namespace ai {

SideAttacker::SideAttacker(string name, bt::Blackboard::Ptr blackboard)
        : Skill(std::move(name), std::move(blackboard)) {
}

void SideAttacker::onInitialize() {
    coach::g_offensiveCoach.addSideAttacker(robot);
}

/// Get an update on the skill
bt::Node::Status SideAttacker::onUpdate() {
    targetPos = getOffensivePosition();
    auto newPosition = goToPos.getPosVelAngle(robot, targetPos);
    Vector2 velocity = newPosition.vel;
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    command.w = static_cast<float>(newPosition.angle);

    command.use_angle = 1;
    publishRobotCommand();
    status = Status::Running;
    return status;
}

Vector2 SideAttacker::getOffensivePosition() {
    return coach::g_offensiveCoach.getPositionForRobotID(robot->id);
}

void SideAttacker::onTerminate(Status s) {
    command.w = static_cast<float>(robot->angle);
    command.x_vel = 0;
    command.y_vel = 0;
    coach::g_offensiveCoach.removeSideAttacker(robot);
    publishRobotCommand();
}

    } // ai
} // rtt