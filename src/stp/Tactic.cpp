//
// Created by jessevw on 03.03.20.
//

#include "include/roboteam_ai/stp/Tactic.h"

#include <roboteam_utils/Print.h>

#include "stp/StpInfo.h"

namespace rtt::ai::stp {

void Tactic::initialize() noexcept { onInitialize(); }

Status Tactic::update(StpInfo const &info) noexcept {
    // Check if the skills are all finished
    if (skills.finished()) {
        RTT_INFO("TACTIC SUCCESSFUL!!!!!!!!!!!!!!!!!!!!!!!!:)")
        return Status::Success;
    }

    // Update skill info
    auto skill_info = calculateInfoForSkill(info);

    // Update the current skill with the new SkillInfo
    auto status = skills.update(skill_info);
    RTT_INFO("ID AFTER UPDATE: ", skills.current_num(), " Called on robot: ", info.getRobot()->get()->getId());

    // Call onUpdate on a skill for specific behaviour
    onUpdate(status);

    return Status::Running;
}

void Tactic::terminate() noexcept { onTerminate(); }
}  // namespace rtt::ai::stp
