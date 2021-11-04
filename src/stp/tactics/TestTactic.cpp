//
// Created by roboteam on 9/3/20.
//

#include "stp/tactics/TestTactic.h"

#include "stp/skills/TestSkill.h"

namespace rtt::ai::stp {

TestTactic::TestTactic() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::TestSkill()};
}

std::optional<StpInfo> TestTactic::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;
    if (!skillStpInfo.getField()) return std::nullopt;
    return skillStpInfo;
}

bool TestTactic::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool TestTactic::shouldTacticReset(const StpInfo &info) noexcept { return false; }

bool TestTactic::forceTacticSuccess(const StpInfo &info) noexcept { return false; }

bool TestTactic::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *TestTactic::getName() { return "Test Tactic"; }

}  // namespace rtt::ai::stp
