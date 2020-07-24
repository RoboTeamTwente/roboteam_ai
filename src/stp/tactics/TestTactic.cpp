//
// Created by roboteam on 9/3/20.
//

#include "stp/tactics/TestTactic.h"

#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp {

TestTactic::TestTactic() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::GoToPos(), skill::GoToPos(), skill::GoToPos(), skill::GoToPos()};
}

std::optional<StpInfo> TestTactic::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;
    if (!skillStpInfo.getField()) return std::nullopt;

    auto length = skillStpInfo.getField()->getFieldLength();
    auto width = skillStpInfo.getField()->getFieldWidth();

    // Set positions based on the current_num() of the state machine
    // This switch will let a robot drive in a square
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

const char *TestTactic::getName() { return "Test Tactic"; }

}  // namespace rtt::ai::stp
