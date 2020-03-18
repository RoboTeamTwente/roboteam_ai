//
// Created by thijs on 17-12-18.
//

#include <skills/SideAttacker.h>
#include <coach/OffensiveCoach.h>

namespace rtt::ai {

SideAttacker::SideAttacker(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void SideAttacker::onInitialize() { coach::g_offensiveCoach.addSideAttacker(*field, *robot); }

/// Get an update on the skill
bt::Node::Status SideAttacker::onUpdate() {
    targetPos = getOffensivePosition(*field);
    auto newPosition = robot->getControllers().getNumTreePosController()->getRobotCommand(robot->get()->getId(), targetPos);
    Vector2 velocity = newPosition.vel;
    command.mutable_vel()->set_x(velocity.x);
    command.mutable_vel()->set_y(velocity.y);
    command.set_w(newPosition.angle);

    command.set_use_angle(true);
    publishRobotCommand();
    status = Status::Running;
    return status;
}

Vector2 SideAttacker::getOffensivePosition(const rtt::ai::world::Field &field) { return coach::g_offensiveCoach.getPositionForRobotID(field, robot->get()->getId()); }

void SideAttacker::onTerminate(Status s) {
    command.set_w(robot->get()->getAngle());
    command.mutable_vel()->set_x(0);
    command.mutable_vel()->set_y(0);
    coach::g_offensiveCoach.removeSideAttacker(*robot);
    publishRobotCommand();
}

}  // namespace rtt::ai