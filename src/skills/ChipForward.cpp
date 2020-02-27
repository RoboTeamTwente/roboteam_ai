//
// Created by robzelluf on 7/5/19.
//

#include <skills/ChipForward.h>

#include <world_new/FieldComputations.hpp>

namespace rtt::ai {

ChipForward::ChipForward(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void ChipForward::onInitialize() {
    aimPoint = world_new::FieldComputations::getPenaltyPoint(*field, false);
    hasChipped = false;
}

ChipForward::Status ChipForward::onUpdate() {
    auto shotData = robot->getControllers().getShotController()->getRobotCommand(*field, *robot, aimPoint, true,
                                                                                 control::BallSpeed::MAX_SPEED,
                                                                                 control::ShotPrecision::LOW,
                                                                                 <#initializer#>, <#initializer#>);
    command = shotData.makeROSCommand();
    if (!hasChipped && command.chipper()) {
        if (command.chip_kick_forced() || robot->hasBall()) hasChipped = true;
    }

    publishRobotCommand();

    if (hasChipped && (ball->get()->getPos() - robot->get()->getPos()).length() > 0.5) {
        return Status::Success;
    }

    return Status::Running;
}

}  // namespace rtt::ai
