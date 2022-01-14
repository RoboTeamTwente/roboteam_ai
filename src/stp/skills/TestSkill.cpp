//
// Created by rtt-vision on 12-10-21.
//

#include "stp/skills/TestSkill.h"

#include <roboteam_utils/Print.h>


namespace rtt::ai::stp::skill {

Status TestSkill::onUpdate(const StpInfo &info) noexcept {
    RTT_DEBUG("pos: ", info.getRobot()->get()->getPos());
    forwardRobotCommand(info.getCurrentWorld());
    return Status::Running;
}

const char *TestSkill::getName() { return "TestSkill"; }

}  // namespace rtt::ai::stp::skill
