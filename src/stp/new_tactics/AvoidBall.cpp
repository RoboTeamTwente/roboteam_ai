//
// Created by jesse on 30-04-20.
//

#include "stp/new_tactics/AvoidBall.h"

#include "stp/invariants/game_states/StopGameStateInvariant.h"
#include "stp/new_skills/GoToPos.h"
#include "utilities/GameStateManager.hpp"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::tactic {

AvoidBall::AvoidBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

void AvoidBall::onInitialize() noexcept {}

void AvoidBall::onUpdate(Status const &status) noexcept {}

void AvoidBall::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

StpInfo AvoidBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    // Code for stop state is here
    if (GameStateManager::getCurrentGameState().getStrategyName() == "stop") {
        auto ballPosition = info.getBall()->get()->getPos();
        auto targetPosition = info.getPositionToMoveTo().value();

        if (targetPosition.dist(ballPosition) < 0.5) {
            skillStpInfo.setPositionToMoveTo(ballPosition.scale(0.7));
        }
    }

    // Code for ballplacement is here
    // If the robot is not colinear with ballplacement path
    else {
        if (rtt::distanceFromPointToLine(info.getBall()->get()->getPos(), rtt::ai::GameStateManager::getRefereeDesignatedPosition(), info.getRobot()->get()->getPos()) < 0.5) {
            LineSegment lineSegment = LineSegment(info.getBall().value().get()->getPos(), rtt::ai::GameStateManager::getRefereeDesignatedPosition());
            auto a = info.getRobot()->get()->getPos().project(info.getBall().value().get()->getPos(), rtt::ai::GameStateManager::getRefereeDesignatedPosition());
            auto c = info.getRobot().value()->getPos();
            skillStpInfo.setPositionToMoveTo(a + (a - c).stretchToLength(0.9));
        }
    }
    // TODO: fix if robot is colinear

    return skillStpInfo;
}

bool AvoidBall::isEndTactic() noexcept { return true; }

bool AvoidBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool AvoidBall::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

const char *AvoidBall::getName() { return "Avoid Ball"; }

}  // namespace rtt::ai::stp::tactic
