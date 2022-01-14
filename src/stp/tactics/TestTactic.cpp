//
// Created by roboteam on 9/3/20.
//

#include "stp/tactics/TestTactic.h"

#include "roboteam_utils/Print.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Kick.h"
#include "stp/skills/TestSkill.h"

namespace rtt::ai::stp {

TestTactic::TestTactic() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Kick()};
}

std::optional<StpInfo> TestTactic::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;
    if (!skillStpInfo.getField()) return std::nullopt;

    skillStpInfo.setPositionToMoveTo(skillStpInfo.getBall()->get()->getPos() - ((info.getBall()->get()->getPos() - info.getRobot()->get()->getPos()).stretchToLength(0.022)));
    skillStpInfo.setAngle((skillStpInfo.getBall()->get()->getPos() - info.getRobot()->get()->getPos()).angle());

    skillStpInfo.setKickChipVelocity(6.5);

    return skillStpInfo;
}

bool TestTactic::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool TestTactic::shouldTacticReset(const StpInfo &info) noexcept { return info.getRobot()->hasBall(); }

bool TestTactic::forceTacticSuccess(const StpInfo &info) noexcept { return info.getRobot()->hasBall(); }

bool TestTactic::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *TestTactic::getName() { return "Test Tactic"; }

}  // namespace rtt::ai::stp
