//
// Created by thijs on 15-5-19.
//

#include <skills/BallPlacementWithInterface.h>
#include <interface/api/Output.h>

namespace rtt::ai {
BallPlacementWithInterface::BallPlacementWithInterface(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {}

Skill::Status BallPlacementWithInterface::onUpdate() {
    if (interface::Output::usesRefereeCommands()) {
        return Status::Failure;
    }

    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();

    auto robotCommand = robot->getControllers().getBallHandlePosController()->getRobotCommand(robot->get()->getId(), targetPos, robot->get()->getAngle(),
                                                                                              control::BallHandlePosControl::TravelStrategy::FORWARDS);

    if (targetPos == previousTargetPos && robot->getControllers().getBallHandlePosController()->getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
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

}  // namespace rtt::ai