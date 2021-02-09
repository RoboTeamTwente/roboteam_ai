//
// Created by timovdk on 5/12/20.
/// Will position the Robot between the BALL and either:
/// - The shot line of the ENEMY with the ball, if ENEMY has ball
/// - Our goal, if no-one has the ball
/// TODO-Max Split this.

/// PASSIVE
//

#include "include/roboteam_ai/stp/tactics/passive/BlockBall.h"

#include <roboteam_utils/Circle.h>

#include "control/ControlUtils.h"
#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

BlockBall::BlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> BlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getField() || !skillStpInfo.getBall() || !skillStpInfo.getRobot() || !skillStpInfo.getEnemyRobot()) return std::nullopt;

    auto field = info.getField().value();
    auto ball = info.getBall().value();
    auto robot = info.getRobot().value();
    auto enemyRobot = info.getEnemyRobot().value();

    auto robotToBall = ball->getPos() - robot->getPos();

    auto targetPosition = calculateTargetPosition(ball, field, enemyRobot);

    auto targetAngle = robotToBall.angle();

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

const char *BlockBall::getName() { return "Block Ball"; }

Vector2 BlockBall::calculateTargetPosition(const world::view::BallView &ball, const world::Field &field, const world::view::RobotView &enemyRobot) noexcept {
    Vector2 targetPosition{};

    // Opponent is close to ball
    // Block the ball by staying on the shot line of the opponent
    if (enemyRobot->getDistanceToBall() < control_constants::ENEMY_CLOSE_TO_BALL_DISTANCE) {
        // TODO: Tune this distance
        const double BLOCK_DISTANCE(0.5);

        auto ballCircle = Circle(ball->getPos(), BLOCK_DISTANCE);

        // Project the enemy robot on the circle, and rotate this position with Pi.
        // This is to stand directly opposite the enemy robot with the ball in the middle.
        targetPosition = ballCircle.project(enemyRobot->getPos()).rotateAroundPoint(M_PI, ballCircle.center);
    }

    // Default
    // Stay between the ball and the center of the goal
    else {
        // TODO: Tune this distance
        const double BLOCK_DISTANCE(0.5);

        auto ballCircle = Circle(ball->getPos(), BLOCK_DISTANCE);

        // Project our goal center on the circle to get the position to block
        targetPosition = ballCircle.project(field.getOurGoalCenter());
    }

    return targetPosition;
}

}  // namespace rtt::ai::stp::tactic