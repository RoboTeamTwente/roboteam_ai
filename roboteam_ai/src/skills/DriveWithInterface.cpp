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

    if ((targetPos - robot->pos).length() < 0.15) {
        publishRobotCommand();
        return Status::Running;
    }

    auto pva = numTreeGtp.getPosVelAngle(robot, targetPos);
    Vector2 velocity = control::ControlUtils::velocityLimiter(pva.vel, Constants::MAX_VEL());
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    command.w = pva.angle;
    publishRobotCommand();
    return Status::Running;
}
}
}