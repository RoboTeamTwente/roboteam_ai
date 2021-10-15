//
// Created by rtt-vision on 12-10-21.
//

#include "stp/skills/TestSkill.h"

namespace rtt::ai::stp::skill {

Status TestSkill::onUpdate(const StpInfo &info) noexcept {
    command.set_dribbler(info.getDribblerSpeed());
    auto targetAngle = info.getAngle();
    command.set_w(static_cast<float>(targetAngle));
    forwardRobotCommand(info.getCurrentWorld());
    return Status::Running;
}

const char *TestSkill::getName() { return "TestSkill"; }

}  // namespace rtt::ai::stp::skill
