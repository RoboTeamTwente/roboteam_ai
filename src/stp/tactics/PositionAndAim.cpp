//
// Created by jordi on 20-05-20.
/// TODO-Max remove, made from other blocks (getBallInDirection?)
//

#include "stp/tactics/PositionAndAim.h"

#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

PositionAndAim::PositionAndAim() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

std::optional<StpInfo> PositionAndAim::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;
    if (!skillStpInfo.getPositionToShootAt() && !skillStpInfo.getRobot()) {
        return std::nullopt;
    }
    skillStpInfo.setAngle(Angle(info.getPositionToShootAt().value() - info.getRobot().value()->getPos()));

    return skillStpInfo;
}

bool PositionAndAim::isTacticFailing(const StpInfo &info) noexcept {
    // Tactic fails if there is no location to move to or no location to aim to
    return !info.getPositionToMoveTo() || !info.getPositionToShootAt();
}

bool PositionAndAim::shouldTacticReset(const StpInfo &info) noexcept {
    // Tactic resets when robot position is not close enough to the target position
    double errorMargin = stp::control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

bool PositionAndAim::isEndTactic() noexcept {
    // Tactic is not an end tactic
    return true;
}

const char *PositionAndAim::getName() { return "Position And Aim"; }

}  // namespace rtt::ai::stp::tactic