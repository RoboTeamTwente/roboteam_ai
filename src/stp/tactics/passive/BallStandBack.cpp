//
// Created by agata on 29/06/2022.
//

#include "stp/skills/GoToPos.h"
#include "stp/tactics/passive/BallStandBack.h"

namespace rtt::ai::stp::tactic {

BallStandBack::BallStandBack() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()};
}

std::optional<StpInfo> BallStandBack::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!info.getPositionToMoveTo() || !skillStpInfo.getBall() || !skillStpInfo.getRobot()) return std::nullopt;

    Vector2 targetPos;
    if (standStillCounter > 60) {
        auto moveVector = info.getRobot()->get()->getPos() - info.getBall()->get()->position;
        targetPos = info.getBall()->get()->position + moveVector.stretchToLength(0.30);
    } else {
        standStillCounter++;
        targetPos = info.getRobot()->get()->getPos();
    }

    double angle = (info.getBall()->get()->position - targetPos).angle();
    skillStpInfo.setPositionToMoveTo(targetPos);
    skillStpInfo.setAngle(angle);

    // Be 100% sure the dribbler is off during the BallStandBack
    skillStpInfo.setDribblerSpeed(0);
    skillStpInfo.setShouldAvoidBall(true);
    return skillStpInfo;
}

bool BallStandBack::isTacticFailing(const StpInfo &info) noexcept {
    // BallStandBack tactic fails if there is no location to move to
    return !info.getPositionToMoveTo();
}

bool BallStandBack::shouldTacticReset(const StpInfo &info) noexcept {
    bool shouldReset = (info.getRobot()->get()->getPos() - info.getBall()->get()->position).length() < 0.15;
    if (!shouldReset) standStillCounter = 0;
    return shouldReset;
}

bool BallStandBack::isEndTactic() noexcept {
    // BallStandBack tactic is an end tactic
    return false;
}

const char *BallStandBack::getName() { return "BallStandBack"; }

}  // namespace rtt::ai::stp::tactic
