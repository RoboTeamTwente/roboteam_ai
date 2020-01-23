//
// Created by baris on 10-5-19.
//

#include "skills/DriveWithInterface.h"

#include <interface/api/Output.h>

namespace rtt::ai {
DriveWithInterface::DriveWithInterface(string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {}
Skill::Status DriveWithInterface::onUpdate() {
    if (interface::Output::usesRefereeCommands()) {
        return Status::Failure;
    }
    //TODO: workaround until world ownership changes
    if (numTreeGtp == nullptr){
        numTreeGtp = new control::PositionControl(*world, *field);
    }
    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();
    auto robotCommand = numTreeGtp->computeAndTrackPath(robot->id, robot->pos,
                                                       robot->vel, targetPos);

    command.mutable_vel()->set_x(robotCommand.vel.x);
    command.mutable_vel()->set_y(robotCommand.vel.y);
    command.set_w(robotCommand.angle);
    publishRobotCommand();
    return Status::Running;
}
}  // namespace rtt