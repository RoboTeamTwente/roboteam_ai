//
// Created by baris on 10-5-19.
//

#include <roboteam_ai/src/interface/api/Output.h>
#include "DriveWithInterface.h"

namespace rtt {
namespace ai {
DriveWithInterface::DriveWithInterface(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
Skill::Status DriveWithInterface::onUpdate() {


    if (interface::Output::usesRefereeCommands()) {
        return Status::Failure;
    }
    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();

    auto robotCommand = numTreeGtp.getPosVelAngle(robot, targetPos);
    command.x_vel = static_cast<float>(robotCommand.vel.x);
    command.y_vel = static_cast<float>(robotCommand.vel.y);
    command.w = robotCommand.angle;
    publishRobotCommand();
    return Status::Running;
}
}
}