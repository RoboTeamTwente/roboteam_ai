//
// Created by jesse on 30-04-20.
//

#include "stp/new_tactics/AvoidBall.h"

#include "roboteam_utils/Tube.h"
#include "stp/invariants/game_states/StopGameStateInvariant.h"
#include "stp/new_skills/GoToPos.h"
#include "utilities/GameStateManager.hpp"

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
    auto currentGameState = GameStateManager::getCurrentGameState().getStrategyName();

    // If gameState == stop we need to avoid using a circle around the ball
    if (currentGameState == "stop") {
        auto ballPosition = skillStpInfo.getBall()->get()->getPos();
        auto targetPosition = skillStpInfo.getPositionToMoveTo().value();

        // Circle around the ball with default avoid radius (0.5m)
        auto avoidCircle = Circle(ballPosition, control_constants::AVOID_BALL_DISTANCE);

        // If position is within the avoidCircle, project the position on this circle
        if (avoidCircle.doesIntersectOrContain(targetPosition)) {
            skillStpInfo.setPositionToMoveTo(avoidCircle.project(targetPosition));
        }
    }

    // If gameState == ballPlacement we need to avoid using a tube on the shortest path from the ball to the ball placement position
    else if (currentGameState == "ball_placement_us" || currentGameState == "ball_placement_them") {
        auto ballPosition = skillStpInfo.getBall()->get()->getPos();
        auto targetPosition = skillStpInfo.getPositionToMoveTo().value();

        // Tube around the shortest path from ball to placement position with default avoid radius (0.5m)
        auto avoidTube = Tube(ballPosition, GameStateManager::getRefereeDesignatedPosition(), control_constants::AVOID_BALL_DISTANCE);

        // If position is within the avoidTube, project the position on this tube
        if (avoidTube.contains(targetPosition)) {
            skillStpInfo.setPositionToMoveTo(avoidTube.project(targetPosition));
        }
    }

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
