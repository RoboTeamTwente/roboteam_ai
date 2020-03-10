//
// Created by jessevw on 03.03.20.
//

#include "include/roboteam_ai/stp/Tactic.h"
#include "stp/StpInfo.h"

namespace rtt::ai::stp {

void Tactic::initialize() noexcept {
    onInitialize();
}

Status Tactic::update(TacticInfo const &info) noexcept {
    // Check if the skills are all finished
    if(skills.finished()) {
        return Status::Success;
    }

    // Update skill info
    auto skill_info = calculateInfoForSkill(info);

    // Update the current skill with the new SkillInfo
    auto status = skills.update(skill_info);

    // Call onUpdate on a skill for specific behaviour
    onUpdate(status);

    return status;
}

void Tactic::terminate() noexcept {
    onTerminate();
}
}  // namespace rtt::ai::stp
