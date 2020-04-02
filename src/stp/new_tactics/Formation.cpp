//
// Created by timovdk on 3/27/20.
//

#include "stp/new_tactics/Formation.h"
#include "stp/new_skills/GoToPos.h"
#include "stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {

Formation::Formation() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

void Formation::onInitialize() noexcept {}

void Formation::onUpdate(Status const &status) noexcept {}

void Formation::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

StpInfo Formation::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    // Be 100% sure the angle is 0 during the formation
    skillStpInfo.setAngle(0);

    // Be 100% sure the dribbler is of during the formation
    skillStpInfo.setDribblerSpeed(0);

    return skillStpInfo;
}

bool Formation::isTacticFailing(const StpInfo &info) noexcept {
    // Formation tactic fails if there is no location to move to
    return !info.getPositionToMoveTo();
}

bool Formation::shouldTacticReset(const StpInfo &info) noexcept {
    // Formation tactic resets when robot position is not close enough to the target position
    double errorMargin = stp::control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

bool Formation::isEndTactic() noexcept {
    // Formation tactic is an end tactic
    return true;
}

const char *Formation::getName() {
    return "Formation";
}

}  // namespace rtt::ai::stp::tactic