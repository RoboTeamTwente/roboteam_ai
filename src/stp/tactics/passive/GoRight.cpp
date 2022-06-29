//
// Created by agata on 29/06/2022.
//

#include "stp/tactics/passive/GoRight.h"

#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

GoRight::GoRight() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()};
}

std::optional<StpInfo> GoRight::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!info.getPositionToMoveTo()) return std::nullopt;

    auto positionToMove = Vector2(info.getPositionToMoveTo().value().x + info.getField()->getFieldLength() / 3, 0);

    skillStpInfo.setPositionToMoveTo(positionToMove);

    // 0 dribbler off - 100 dribbler on
    skillStpInfo.setDribblerSpeed(0);

    skillStpInfo.setAngle(0);

    return skillStpInfo;
}

bool GoRight::isTacticFailing(const StpInfo &info) noexcept {
    // GoRight tactic fails if there is no location to move to
    return !info.getPositionToMoveTo();
}

bool GoRight::shouldTacticReset(const StpInfo &info) noexcept { return false; }

bool GoRight::isEndTactic() noexcept {
    // GoRight tactic is an end tactic
    return false;
}

const char *GoRight::getName() { return "GoRight"; }

}  // namespace rtt::ai::stp::tactic