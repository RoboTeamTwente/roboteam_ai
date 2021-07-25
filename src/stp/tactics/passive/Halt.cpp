//
// Created by jessevw on 24.03.20.
/// ROTATES robot to face forwards
//

#include "stp/tactics/passive/Halt.h"

#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

Halt::Halt() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::Rotate()}; }

std::optional<StpInfo> Halt::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillInfo = info;
    skillInfo.setAngle(0.00001);

    return skillInfo;
}

bool Halt::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool Halt::shouldTacticReset(const StpInfo &info) noexcept { return false; }

bool Halt::isEndTactic() noexcept {
    // This is an end tactic
    return true;
}

const char *Halt::getName() { return "Halt"; }

}  // namespace rtt::ai::stp::tactic
