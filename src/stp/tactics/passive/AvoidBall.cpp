//
// Created by jesse on 30-04-20.
/// Draws a circle around the ball, Robot will stay out of the circle (getting pushed back). Angle is forced on 0 degrees

/// PASSIVE
//

#include "stp/tactics/passive/AvoidBall.h"

#include <roboteam_utils/Circle.h>
#include <roboteam_utils/Tube.h>

#include "stp/computations/PositionComputations.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::tactic {

AvoidBall::AvoidBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()}; }

std::optional<StpInfo> AvoidBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;
    auto currentGameState = GameStateManager::getCurrentGameState().getStrategyName();

    if (!skillStpInfo.getBall() || !skillStpInfo.getRobot()) return std::nullopt;
    if (!skillStpInfo.getPositionToMoveTo()) skillStpInfo.setPositionToMoveTo(info.getRobot()->get()->getPos());
    skillStpInfo.setDribblerSpeed(0);

    Vector2 targetPos = skillStpInfo.getPositionToMoveTo().value();

    // If gameState == stop we need to avoid using a circle around the ball
    if (std::strcmp(currentGameState.c_str(), "stop") == 0) {
        auto ballPosition = skillStpInfo.getBall()->get()->getPos();
        auto targetPosition = skillStpInfo.getPositionToMoveTo().value();

        // Circle around the ball with default avoid radius (0.5m)
        auto avoidCircle = Circle(ballPosition, control_constants::AVOID_BALL_DISTANCE);

        // If position is within the avoidCircle, project the position on this circle
        if (avoidCircle.doesIntersectOrContain(targetPosition)) {
            targetPos = avoidCircle.project(targetPosition);
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
            targetPos = avoidTube.project(targetPosition);
        }
    }

    targetPos =
        control::ControlUtils::projectPointToValidPosition(info.getField().value(), targetPos, skillStpInfo.getRoleName(), control_constants::DEFENSE_AREA_AVOIDANCE_MARGIN);

    skillStpInfo.setPositionToMoveTo(targetPos);

    return skillStpInfo;
}

bool AvoidBall::isEndTactic() noexcept { return true; }

bool AvoidBall::isTacticFailing(const StpInfo &info) noexcept {
    return (info.getRoleName() == "ball_placer" &&
            (info.getBall()->get()->getPos() - rtt::ai::GameStateManager::getRefereeDesignatedPosition()).length() > control_constants::BALL_PLACEMENT_MARGIN);
}

bool AvoidBall::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

const char *AvoidBall::getName() { return "Avoid Ball"; }

}  // namespace rtt::ai::stp::tactic
