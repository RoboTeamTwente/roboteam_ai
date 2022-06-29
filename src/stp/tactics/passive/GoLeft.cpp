//
// Created by agata on 29/06/2022.
//

#include "stp/tactics/passive/GoLeft.h"

#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

GoLeft::GoLeft() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()};
}

std::optional<StpInfo> GoLeft::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!info.getPositionToMoveTo()) return std::nullopt;

    auto positionToMove = Vector2(info.getPositionToMoveTo().value().x - info.getField()->getFieldLength() / 3, 0);

    skillStpInfo.setPositionToMoveTo(positionToMove);

    // 0 dribbler off - 100 dribbler on
    skillStpInfo.setDribblerSpeed(0);

    skillStpInfo.setAngle(M_PI / 2);

    return skillStpInfo;
}

bool GoLeft::isTacticFailing(const StpInfo &info) noexcept {
    // GoLeft tactic fails if there is no location to move to
    return !info.getPositionToMoveTo();
}

bool GoLeft::shouldTacticReset(const StpInfo &info) noexcept { return false; }

bool GoLeft::isEndTactic() noexcept {
    // GoLeft tactic is an end tactic
    return false;
}

const char *GoLeft::getName() { return "GoLeft"; }

}  // namespace rtt::ai::stp::tactic