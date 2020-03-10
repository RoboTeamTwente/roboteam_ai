//
// Created by robzelluf on 7/5/19.
//

#include <skills/ChipForward.h>

namespace rtt::ai {

ChipForward::ChipForward(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void ChipForward::onInitialize() {
    aimPoint = FieldComputations::getPenaltyPoint(*field, false);
    hasChipped = false;
}

ChipForward::Status ChipForward::onUpdate() {
    auto shotData = robot->getControllers().getShotController()->getRobotCommand(robot->get()->getId(), aimPoint, true, control::BallSpeed::MAX_SPEED, false, control::ShotPrecision::LOW);
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
