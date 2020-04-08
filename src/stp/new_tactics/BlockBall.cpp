//
// Created by jordi on 08-04-20.
//

#include "stp/new_tactics/BlockBall.h"
#include "stp/new_skills/GoToPos.h"
#include "stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {

BlockBall::BlockBall() {
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

void BlockBall::onInitialize() noexcept {}

void BlockBall::onUpdate(Status const &status) noexcept {}

void BlockBall::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

StpInfo BlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    return skillStpInfo;
}

bool BlockBall::isEndTactic() noexcept { return true; }

bool BlockBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockBall::shouldTacticReset(const StpInfo &info) noexcept { return false; }

const char *BlockBall::getName() {
    return "Block Ball";
}

}  // namespace rtt::ai::stp::tactic
