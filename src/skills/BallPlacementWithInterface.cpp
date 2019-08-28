//
// Created by thijs on 15-5-19.
//

#include "include/roboteam_ai/skills/BallPlacementWithInterface.h"

//
// Created by baris on 10-5-19.
//

#include <include/roboteam_ai/interface/api/Output.h>
#include "include/roboteam_ai/skills/DriveWithInterface.h"

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

    auto robotCommand = ballHandlePosControl.getRobotCommand(robot, targetPos, robot->angle, control::BallHandlePosControl::TravelStrategy::FORWARDS );

    if (targetPos == previousTargetPos &&
        ballHandlePosControl.getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        command.mutable_vel()->set_x(0);
        command.mutable_vel()->set_y(0);
        command.set_w(robotCommand.angle);
        command.set_dribbler(0);
        publishRobotCommand();
        return Status::Running;
    }
    command.mutable_vel()->set_x(static_cast<float>(robotCommand.vel.x));
    command.mutable_vel()->set_y(static_cast<float>(robotCommand.vel.y));
    command.set_w(robotCommand.angle);
    command.set_dribbler(robotCommand.dribbler);
    publishRobotCommand();

    previousTargetPos = targetPos;
    return Status::Running;
}

}
}