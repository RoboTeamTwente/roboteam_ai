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

AvoidBall::AvoidBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> AvoidBall::calculateInfoForSkill(StpInfo const& info) noexcept {
    StpInfo skillStpInfo = info;
    auto currentGameState = GameStateManager::getCurrentGameState().getStrategyName();

    if (!skillStpInfo.getBall() || !skillStpInfo.getRobot()) return std::nullopt;
    if (!skillStpInfo.getPositionToMoveTo()) skillStpInfo.setPositionToMoveTo(info.getRobot()->get()->getPos());
    skillStpInfo.setDribblerSpeed(0);

    Vector2 targetPos = skillStpInfo.getPositionToMoveTo().value();
    auto ballPosition = skillStpInfo.getBall()->get()->position;

    std::unique_ptr<Shape> avoidShape;

    // During ball placement, we need to avoid the are between the ball and the target position by a certain margin
    if (currentGameState == "ball_placement_us" || currentGameState == "ball_placement_them") {
        avoidShape = std::make_unique<Tube>(Tube(ballPosition, GameStateManager::getRefereeDesignatedPosition(), control_constants::AVOID_BALL_DISTANCE));
    } else {
        // During stop gamestate, we need to avoid the area directly around the ball.
        avoidShape = std::make_unique<Circle>(Circle(ballPosition, control_constants::AVOID_BALL_DISTANCE));
    }

    if (avoidShape->contains(targetPos)) {
        auto projectedPos = avoidShape->project(targetPos);
        if (FieldComputations::pointIsValidPosition(info.getField().value(), projectedPos))
            targetPos = projectedPos;
        else {
            targetPos = calculateNewPosition(ballPosition, info.getField().value(), avoidShape);
        }
    }

    skillStpInfo.setPositionToMoveTo(targetPos);
    skillStpInfo.setShouldAvoidBall(true);
    return skillStpInfo;
}

bool AvoidBall::isEndTactic() noexcept { return true; }

bool AvoidBall::isTacticFailing(const StpInfo& info) noexcept {
    return (info.getRoleName() == "ball_placer" &&
            (info.getBall()->get()->position - rtt::ai::GameStateManager::getRefereeDesignatedPosition()).length() > control_constants::BALL_PLACEMENT_MARGIN);
}

bool AvoidBall::shouldTacticReset(const StpInfo& info) noexcept { return false; }

Vector2 AvoidBall::calculateNewPosition(Vector2 ballPos, const rtt::world::Field& field, const std::unique_ptr<Shape>& avoidShape) {
    Vector2 newTarget = ballPos;  // The new position to go to
    bool pointFound = false;
    for (int distanceSteps = 0; distanceSteps < 5; ++distanceSteps) {
        // Use a larger grid each iteration in case no valid point is found
        auto distance = 3 * control_constants::AVOID_BALL_DISTANCE + distanceSteps * control_constants::AVOID_BALL_DISTANCE / 2.0;
        auto possiblePoints = Grid(ballPos.x - distance / 2.0, ballPos.y - distance / 2.0, distance, distance, 3, 3).getPoints();
        double dist = 1e3;
        for (auto& pointVector : possiblePoints) {
            for (auto& point : pointVector) {
                if (FieldComputations::pointIsValidPosition(field, point) && !avoidShape->contains(point)) {
                    if (ballPos.dist(point) < dist) {
                        dist = ballPos.dist(point);
                        newTarget = point;
                        pointFound = true;
                    }
                }
            }
        }
        if (pointFound) break;  // As soon as a valid point is found, don't look at more points further away
    }
    if (newTarget == ballPos) RTT_WARNING("Could not find good position to avoid ball");
    return newTarget;
}

const char* AvoidBall::getName() { return "Avoid Ball"; }

}  // namespace rtt::ai::stp::tactic
