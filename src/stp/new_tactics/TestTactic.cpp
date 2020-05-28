//
// Created by roboteam on 9/3/20.
//

#include "stp/new_tactics/TestTactic.h"

#include <stp/new_skills/GoToPos.h>
#include <stp/new_skills/Rotate.h>

namespace rtt::ai::stp {

TestTactic::TestTactic() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::GoToPos(), skill::GoToPos(), skill::GoToPos(), skill::GoToPos()};
}

void TestTactic::onInitialize() noexcept {}

void TestTactic::onUpdate(Status const &status) noexcept {}

void TestTactic::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

StpInfo TestTactic::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    auto length = skillStpInfo.getField()->getFieldLength();
    auto width = skillStpInfo.getField()->getFieldWidth();

    switch (skills.current_num()) {
    case 0:
        skillStpInfo.setPositionToMoveTo(Vector2(length / 8, width / 8));
        break;
    case 1:
        skillStpInfo.setPositionToMoveTo(Vector2(length / 8, -width / 8));
        break;
    case 2:
        skillStpInfo.setPositionToMoveTo(Vector2(-length / 8, -width / 8));
        break;
    case 3:
        skillStpInfo.setPositionToMoveTo(Vector2(-length / 8, width / 8));
        break;
    case 4:
        skillStpInfo.setPositionToMoveTo(Vector2(length / 8, width / 8));
        skills.reset();
        break;
    }


    return skillStpInfo;
}

bool TestTactic::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool TestTactic::shouldTacticReset(const StpInfo &info) noexcept { return false; }

bool TestTactic::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *TestTactic::getName() {
    return "Test Tactic";
}

}  // namespace rtt::ai::stp
