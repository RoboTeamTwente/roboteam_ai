//
// Created by thijs on 17-12-18.
//

#include "skills/SideAttacker.h"

namespace rtt {
namespace ai {

SideAttacker::SideAttacker(string name, bt::Blackboard::Ptr blackboard)
        : Skill(std::move(name), std::move(blackboard)) {
}

void SideAttacker::onInitialize() {
    coach::g_offensiveCoach.addSideAttacker(*field, robot);
}

/// Get an update on the skill
bt::Node::Status SideAttacker::onUpdate() {
    targetPos = getOffensivePosition(*field);
    auto newPosition = robot->getNumtreePosControl()->getRobotCommand(world, field, robot, targetPos);
    Vector2 velocity = newPosition.vel;
    command.mutable_vel()->set_x(velocity.x);
    command.mutable_vel()->set_y(velocity.y);
    command.set_w(newPosition.angle);

    command.set_use_angle(true);
    publishRobotCommand();
    status = Status::Running;
    return status;
}

Vector2 SideAttacker::getOffensivePosition(const Field &field) {
    return coach::g_offensiveCoach.getPositionForRobotID(field, robot->id);
}

void SideAttacker::onTerminate(Status s) {
    command.set_w(robot->angle);
    command.mutable_vel()->set_x(0);
    command.mutable_vel()->set_y(0);
    coach::g_offensiveCoach.removeSideAttacker(robot);
    publishRobotCommand();
}

    } // ai
} // rtt