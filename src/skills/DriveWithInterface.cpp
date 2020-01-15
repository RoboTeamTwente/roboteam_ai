//
// Created by baris on 10-5-19.
//

#include <interface/api/Output.h>
#include "skills/DriveWithInterface.h"

namespace rtt {
namespace ai {
DriveWithInterface::DriveWithInterface(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
    numTreeGtp = new control::PositionControl(*world, *field);
}
Skill::Status DriveWithInterface::onUpdate() {
    if (interface::Output::usesRefereeCommands()) {
        return Status::Failure;
    }
    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();
    auto robotCommand = numTreeGtp.computeAndTrackPath(robot->id, robot->pos,
                                                       robot->vel, targetPos);

    command.mutable_vel()->set_x(robotCommand.vel.x);
    command.mutable_vel()->set_y(robotCommand.vel.y);
    command.set_w(robotCommand.angle);
    publishRobotCommand();
    return Status::Running;
}
}
}