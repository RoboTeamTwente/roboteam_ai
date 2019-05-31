//
// Created by thijs on 15-5-19.
//

#include "BallPlacementWithInterface.h"

//
// Created by baris on 10-5-19.
//

#include <roboteam_ai/src/interface/api/Output.h>
#include "DriveWithInterface.h"

namespace rtt {
namespace ai {
BallPlacementWithInterface::BallPlacementWithInterface(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}

Skill::Status BallPlacementWithInterface::onUpdate() {

    if (interface::Output::usesRefereeCommands()) {
        return Status::Failure;
    }

    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();

    if (targetPos == previousTargetPos &&
        ballHandlePosControl.getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        command.x_vel = 0;
        command.y_vel = 0;
        command.dribbler = 0;
        publishRobotCommand();
        return Status::Running;
    }
    auto rc = ballHandlePosControl.getRobotCommand(robot, targetPos, robot->angle);
    command.x_vel = static_cast<float>(rc.vel.x);
    command.y_vel = static_cast<float>(rc.vel.y);
    command.w = rc.angle;
    command.dribbler = rc.dribbler;
    publishRobotCommand();

    previousTargetPos = targetPos;
    return Status::Running;
}

}
}