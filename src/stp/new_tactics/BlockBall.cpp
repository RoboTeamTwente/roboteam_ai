//
// Created by jordi on 08-04-20.
//

#include "stp/new_tactics/BlockBall.h"
#include "stp/new_skills/GoToPos.h"
#include "stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {

BlockBall::BlockBall() {
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

void BlockBall::onInitialize() noexcept {}

void BlockBall::onUpdate(Status const &status) noexcept {}

void BlockBall::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

StpInfo BlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    auto goalToBall = info.getBall().value()->getPos() - info.getField().value().getOurGoalCenter();
    auto targetPosition = info.getField().value().getOurGoalCenter() + goalToBall.stretchToLength(info.getField().value().getGoalWidth() / 2.0);
    auto targetAngle = goalToBall.angle();

    skillStpInfo.setPositionToMoveTo(targetPosition);
    skillStpInfo.setAngle(targetAngle);

    return skillStpInfo;
}

bool BlockBall::isEndTactic() noexcept { return true; }

bool BlockBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockBall::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

const char *BlockBall::getName() {
    return "Block Ball";
}

}  // namespace rtt::ai::stp::tactic
