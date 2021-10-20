//
// Created by jordi on 08-04-20.
//

#include "stp/tactics/KeeperBlockBall.h"

#include "control/ControlUtils.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

KeeperBlockBall::KeeperBlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> KeeperBlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getField() || !skillStpInfo.getBall() || !skillStpInfo.getRobot()) return std::nullopt;

    auto field = info.getField().value();
    auto ball = info.getBall().value();
    if (!skillStpInfo.getEnemyRobot()){
        skillStpInfo.setPositionToMoveTo(Vector2(field.getOurGoalCenter().x + 0.2, 0));
        return skillStpInfo;
    }

    auto enemyRobot = info.getEnemyRobot().value();


    auto targetPosition = calculateTargetPosition(ball, field, enemyRobot);

    skillStpInfo.setPositionToMoveTo(Vector2(field.getOurGoalCenter().x + 0.2,targetPosition.first.y));
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

    // Ball is on our side
    if (ball->getPos().x < 0) {
        auto keeperArc = Arc(field.getOurGoalCenter(), DISTANCE_FROM_GOAL_FAR, -M_PI / 2, M_PI / 2);

        // Ball is moving
        // Intercept ball when it is moving towards the goal
        if (ball->getVelocity().length() > control_constants::BALL_STILL_VEL) {
            auto start = ball->getPos();
            auto end = start + ball->getVelocity().stretchToLength(field.getFieldLength());
            auto startGoal = field.getOurTopGoalSide() + Vector2(control_constants::DISTANCE_FROM_GOAL_CLOSE, 0);
            auto endGoal = field.getOurBottomGoalSide() + Vector2(control_constants::DISTANCE_FROM_GOAL_CLOSE, 0);

            auto intersection = LineSegment(start, end).intersects(LineSegment(startGoal, endGoal));
            if (intersection) {
                return std::make_pair(intersection.value(), PIDType::KEEPER);
            }
        }

        // Opponent is close to ball
        // Block the ball by staying on the shot line of the opponent
        if (enemyRobot->getDistanceToBall() < control_constants::ENEMY_CLOSE_TO_BALL_DISTANCE) {
            auto start = enemyRobot->getPos();
            auto robotAngle = enemyRobot->getAngle();
            auto end = start + robotAngle.toVector2().stretchToLength(field.getFieldLength());
            const auto &startGoal = field.getOurTopGoalSide();
            const auto &endGoal = field.getOurBottomGoalSide();

            auto intersection = LineSegment(start, end).intersects(LineSegment(startGoal, endGoal));
            if (intersection) {
                auto targetPositions = keeperArc.intersectionWithLine(start, intersection.value());

                if (targetPositions.first) {
                    return std::make_pair(targetPositions.first.value(), PIDType::DEFAULT);
                } else if (targetPositions.second) {
                    return std::make_pair(targetPositions.second.value(), PIDType::DEFAULT);
                }
            }
        }

        auto targetPosition = Vector2(field.getOurGoalCenter().x,
                                      std::clamp(ball->getPos().y, field.getOurGoalCenter().y - field.getGoalWidth() / 2, field.getOurGoalCenter().y + field.getGoalWidth() / 2));
        return std::make_pair(targetPosition, PIDType::DEFAULT);
    }
}

}  // namespace rtt::ai::stp::tactic
