//
// Created by jordi on 20-05-20.
//

#include "include/roboteam_ai/stp/new_tactics/PositionAndAim.h"

#include "stp/new_skills/GoToPos.h"
#include "stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {

PositionAndAim::PositionAndAim() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

void PositionAndAim::onInitialize() noexcept {}

void PositionAndAim::onUpdate(Status const &status) noexcept {}

void PositionAndAim::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

StpInfo PositionAndAim::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

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
    return false;
}

const char *PositionAndAim::getName() {
    return "Position And Aim";
}

}  // namespace rtt::ai::stp::tactic