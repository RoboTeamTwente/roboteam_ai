//
// Created by roboteam on 9/3/20.
//

#include "stp/tactics/TestTactic.h"

#include "stp/skills/Kick.h"
#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp {

TestTactic::TestTactic() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Kick()};
}

std::optional<StpInfo> TestTactic::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;
    if (!skillStpInfo.getField()) return std::nullopt;
    static int counter = 0;
    if (counter < 60){
        skillStpInfo.setKickChipVelocity(0);
        counter++;
    }
    else {
        skillStpInfo.setKickChipVelocity(stp::control_constants::MAX_KICK_POWER);
        counter=0;
    }
    skillStpInfo.setDribblerSpeed(100);
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
