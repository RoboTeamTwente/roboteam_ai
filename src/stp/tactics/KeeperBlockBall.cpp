//
// Created by jordi on 08-04-20.
//

#include "stp/tactics/KeeperBlockBall.h"

#include "control/ControlUtils.h"
#include "roboteam_utils/LineSegment.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

KeeperBlockBall::KeeperBlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> KeeperBlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getField() || !skillStpInfo.getBall() || !skillStpInfo.getRobot()) return std::nullopt;

    auto field = info.getField().value();
    auto ball = info.getBall().value();
    if (!skillStpInfo.getEnemyRobot()) {
        skillStpInfo.setPositionToMoveTo(Vector2(field.getOurGoalCenter().x + Constants::KEEPER_CENTREGOAL_MARGIN(), 0));
        return skillStpInfo;
    }

    // Look towards ball to ensure ball hits the front assembly to reduce odds of ball reflecting in goal
    auto keeperToBall = ball->position - skillStpInfo.getRobot()->get()->getPos();
    skillStpInfo.setAngle(keeperToBall.angle());

    auto enemyRobot = info.getEnemyRobot().value();

    auto targetPosition = calculateTargetPosition(ball, field, enemyRobot);

    skillStpInfo.setPidType(targetPosition.second);
    skillStpInfo.setPositionToMoveTo(Vector2(field.getOurGoalCenter().x + 0.2, targetPosition.first.y));
    return skillStpInfo;
}

bool KeeperBlockBall::isEndTactic() noexcept { return true; }

bool KeeperBlockBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool KeeperBlockBall::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

const char *KeeperBlockBall::getName() { return "Keeper Block Ball"; }

std::pair<Vector2, stp::PIDType> KeeperBlockBall::calculateTargetPosition(const world::view::BallView &ball, const world::Field &field,
                                                                          const world::view::RobotView &enemyRobot) noexcept {
    const double DISTANCE_FROM_GOAL_FAR = field.getGoalWidth() / 1.5;

    auto keeperArc = Arc(field.getOurGoalCenter(), DISTANCE_FROM_GOAL_FAR, -M_PI / 2, M_PI / 2);

    // Keeper should not move too far to the side, it is not great if the keeper is halfway out of the goal
    auto minKeeperY = field.getOurBottomGoalSide().y + control_constants::DISTANCE_TO_ROBOT_CLOSE;
    auto maxKeeperY = field.getOurTopGoalSide().y - control_constants::DISTANCE_TO_ROBOT_CLOSE;

    // Intercept ball when it is moving towards the goal
    if (ball->velocity.length() > control_constants::BALL_STILL_VEL) {
        auto start = ball->position;
        auto end = start + ball->velocity.stretchToLength(field.getFieldLength());
        // Goal is made a bit "bigger" to ensure robot still moves towards ball trajectory
        // even when it thinks the ball will just miss, as this can be error prone
        auto startGoal = field.getOurTopGoalSide() + Vector2(0, control_constants::DISTANCE_TO_ROBOT_CLOSE);
        auto endGoal = field.getOurBottomGoalSide() - Vector2(0, control_constants::DISTANCE_TO_ROBOT_CLOSE);

        auto intersection = LineSegment(start, end).intersects(LineSegment(startGoal, endGoal));
        if (intersection) {
            // Clamp y between goal to ensure robot does not move out of goal
            intersection.value().y = std::clamp(intersection.value().y, field.getOurBottomGoalSide().y, field.getOurTopGoalSide().y);
            return std::make_pair(intersection.value(), PIDType::DEFAULT);
        }
    }

    // Opponent is close to ball and aiming towards the goal
    // Block the ball by staying on the shot line of the opponent
    if (enemyRobot->getDistanceToBall() < control_constants::ENEMY_CLOSE_TO_BALL_DISTANCE) {
        auto start = enemyRobot->getPos();
        auto robotAngle = enemyRobot->getAngle();
        auto end = start + robotAngle.toVector2().stretchToLength(field.getFieldLength());
        // Goal is made a bit "bigger" to ensure robot still moves towards expected target position
        // even when it thinks the ball will just miss, as this can be error prone
        auto startGoal = field.getOurTopGoalSide() + Vector2(0, control_constants::DISTANCE_TO_ROBOT_CLOSE);
        auto endGoal = field.getOurBottomGoalSide() - Vector2(0, control_constants::DISTANCE_TO_ROBOT_CLOSE);

        auto intersection = LineSegment(start, end).intersects(LineSegment(startGoal, endGoal));
        if (intersection) {
            // Clamp y between goal to ensure robot does not move out of goal
            intersection.value().y = std::clamp(intersection.value().y, minKeeperY, maxKeeperY);

            auto targetPositions = keeperArc.intersectionWithLine(start, intersection.value());

            if (targetPositions.first) {
                return std::make_pair(targetPositions.first.value(), PIDType::DEFAULT);
            } else if (targetPositions.second) {
                return std::make_pair(targetPositions.second.value(), PIDType::DEFAULT);
            }
        }
    }

    // Default positioning, stand at same y as the ball is currently located, clamped between the goal
    auto targetPosition = Vector2(field.getOurGoalCenter().x + control_constants::ROBOT_CLOSE_TO_POINT, std::clamp(ball->position.y, minKeeperY, maxKeeperY));
    return std::make_pair(targetPosition, PIDType::DEFAULT);
}

}  // namespace rtt::ai::stp::tactic
