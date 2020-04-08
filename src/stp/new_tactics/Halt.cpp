//
// Created by jessevw on 24.03.20.
//

#include "stp/new_tactics/Halt.h"
#include "stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {

Halt::Halt() {
    skills = collections::state_machine<Skill, Status, StpInfo>{skill::Rotate()};
}

void Halt::onInitialize() noexcept {}

void Halt::onUpdate(Status const &status) noexcept {}

void Halt::onTerminate() noexcept {}

StpInfo Halt::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillInfo = info;
    skillInfo.setAngle(0);

    return skillInfo;
}

bool Halt::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool Halt::shouldTacticReset(const StpInfo &info) noexcept { return false; }

bool Halt::isEndTactic() noexcept {
    // This is an end tactic
    return true;
}

const char *Halt::getName() {
    return "Halt";
}

}  // namespace rtt::ai::stp::tactic
