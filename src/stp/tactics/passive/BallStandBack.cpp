//
// Created by agata on 29/06/2022.
//

#include "stp/tactics/passive/BallStandBack.h"

#include <stp/constants/ControlConstants.h>

#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

BallStandBack::BallStandBack() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()};
}

std::optional<StpInfo> BallStandBack::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!info.getPositionToMoveTo() || !skillStpInfo.getBall()) return std::nullopt;

    auto moveVector = Vector2(info.getRobot()->get()->getPos() - info.getBall()->get()->getPos());

    auto positionToMove = moveVector.stretchToLength(control_constants::AVOID_BALL_DISTANCE);

    double angle = (info.getBall()->get()->getPos() - positionToMove).angle();

    skillStpInfo.setAngle(angle);
    skillStpInfo.setPositionToMoveTo(positionToMove);

    // Be 100% sure the dribbler is off during the BallStandBack
    skillStpInfo.setDribblerSpeed(0);

    return skillStpInfo;
}

bool BallStandBack::isTacticFailing(const StpInfo &info) noexcept {
    // BallStandBack tactic fails if there is no location to move to
    return !info.getPositionToMoveTo();
}

bool BallStandBack::shouldTacticReset(const StpInfo &info) noexcept { return false; }

bool BallStandBack::isEndTactic() noexcept {
    // BallStandBack tactic is an end tactic
    return false;
}

const char *BallStandBack::getName() { return "BallStandBack"; }

}  // namespace rtt::ai::stp::tactic
