//
// Created by baris on 10-5-19.
//

#include "skills/DriveWithInterface.h"

#include <interface/api/Output.h>
#include "world_new/World.hpp"

namespace rtt::ai {
DriveWithInterface::DriveWithInterface(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {}
Skill::Status DriveWithInterface::onUpdate() {
    if (interface::Output::usesRefereeCommands()) {
        return Status::Failure;
    }

    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();
    auto robotCommand = world_new::World::instance()->getRobotPositionController()->computeAndTrackPath(*field, robot->id, robot->pos, robot->vel, targetPos);

    command.mutable_vel()->set_x(robotCommand.vel.x);
    command.mutable_vel()->set_y(robotCommand.vel.y);
    command.set_w(robotCommand.angle);
    publishRobotCommand();
    return Status::Running;
}
}  // namespace rtt::ai