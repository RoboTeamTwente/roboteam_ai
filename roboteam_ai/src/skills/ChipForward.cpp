//
// Created by robzelluf on 7/5/19.
//

#include "ChipForward.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/shotControllers/ShotController.h>

namespace rtt {
namespace ai {

ChipForward::ChipForward(string name, bt::Blackboard::Ptr blackboard)
    :Skill(std::move(name), std::move(blackboard)) {}

void ChipForward::onInitialize() {
    aimPoint = world::field->getPenaltyPoint(false);
    hasChipped = false;
}

ChipForward::Status ChipForward::onUpdate() {
    auto shotData = robot->getShotController()->getRobotCommand(
            *robot, aimPoint, true, control::BallSpeed::MAX_SPEED, false, control::ShotPrecision::LOW);
    command = shotData.makeROSCommand();
    if (!hasChipped && command.chipper == 1) {
        if (command.kicker_forced || robot->hasBall())
        hasChipped = true;
    }

    publishRobotCommand();

    if (hasChipped && (ball->pos - robot->pos).length() > 0.5) {
        return Status::Success;
    }

    return Status::Running;
}

}
}
