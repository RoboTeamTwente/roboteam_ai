//
// Created by maxl on 15-02-21.
//
/// PASSIVE
//

#include "stp/tactics/passive/Walling.h"

#include "stp/constants/ControlConstants.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

Walling::Walling() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

std::optional<StpInfo> Walling::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    // Be 100% sure the dribbler is off during the wall
    skillStpInfo.setDribblerSpeed(100);
    skillStpInfo.setPidType(PIDType::INTERCEPT);
    return skillStpInfo;
}

bool Walling::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool Walling::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

bool Walling::isEndTactic() noexcept {
    // Formation tactic is an end tactic
    return true;
}

const char *Walling::getName() { return "Walling"; }

}  // namespace rtt::ai::stp::tactic