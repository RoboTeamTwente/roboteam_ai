//
// Created by jordi on 08-04-20.
//

#include "stp/new_tactics/KeeperBlockBall.h"

#include "control/ControlUtils.h"
#include "stp/new_skills/GoToPos.h"
#include "stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {

KeeperBlockBall::KeeperBlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()}; }

void KeeperBlockBall::onInitialize() noexcept {}

void KeeperBlockBall::onUpdate(Status const &status) noexcept {}

void KeeperBlockBall::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

std::optional<StpInfo> KeeperBlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getField() || !skillStpInfo.getBall() || !skillStpInfo.getRobot() || !skillStpInfo.getEnemyRobot()) return std::nullopt;

    auto field = info.getField().value();
    auto ball = info.getBall().value();
    auto keeper = info.getRobot().value();
    auto enemyRobot = info.getEnemyRobot().value();

    auto keeperToBall = ball->getPos() - keeper->getPos();

    auto targetPosition = calculateTargetPosition(ball, field, enemyRobot);

    auto targetAngle = keeperToBall.angle();

    skillStpInfo.setPositionToMoveTo(targetPosition.first);
    skillStpInfo.setPidType(targetPosition.second);
    skillStpInfo.setAngle(targetAngle);

    return skillStpInfo;
}

bool KeeperBlockBall::isEndTactic() noexcept { return true; }

bool KeeperBlockBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool KeeperBlockBall::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

const char *KeeperBlockBall::getName() { return "Keeper Block Ball"; }

std::pair<Vector2, stp::PIDType> KeeperBlockBall::calculateTargetPosition(const world_new::view::BallView &ball, const world::Field &field,
                                                                          const world_new::view::RobotView &enemyRobot) noexcept {
    const double DISTANCE_FROM_GOAL_FAR = field.getGoalWidth() / 1.5;

    // Ball is on our side
    if (ball->getPos().x < 0) {
        auto keeperArc = Arc(field.getOurGoalCenter(), DISTANCE_FROM_GOAL_FAR, -M_PI / 2, M_PI / 2);

        // Ball is moving
        // Intercept ball when it is moving towards the goal
        if (ball->getVelocity().length() > control_constants::BALL_STILL_VEL) {
            auto start = ball->getPos();
            auto end = start + ball->getVelocity().stretchToLength(field.getFieldLength() * 0.5);
            auto startGoal = field.getOurTopGoalSide() + Vector2(control_constants::DISTANCE_FROM_GOAL_CLOSE, 0);
            auto endGoal = field.getOurBottomGoalSide() + Vector2(control_constants::DISTANCE_FROM_GOAL_CLOSE, 0);

            auto intersection = LineSegment(start, end).intersects(LineSegment(startGoal, endGoal));
            if (intersection) {
                return std::make_pair(intersection.value(), KEEPER_INTERCEPT);
            }
        }

        // Opponent is close to ball
        // Block the ball by staying on the shot line of the opponent
        if (enemyRobot->getDistanceToBall() < control_constants::ENEMY_CLOSE_TO_BALL_DISTANCE) {
            auto start = enemyRobot->getPos();
            auto enemyToBall = ball->getPos() - start;
            auto end = start + enemyToBall.stretchToLength(field.getFieldLength() * 0.5);
            const auto &startGoal = field.getOurTopGoalSide();
            const auto &endGoal = field.getOurBottomGoalSide();

            auto intersection = LineSegment(start, end).intersects(LineSegment(startGoal, endGoal));
            if (intersection) {
                auto targetPositions = keeperArc.intersectionWithLine(start, intersection.value());

                if (targetPositions.first) {
                    return std::make_pair(targetPositions.first.value(), KEEPER);
                } else if (targetPositions.second) {
                    return std::make_pair(targetPositions.second.value(), KEEPER);
                }
            }
        }

        // Stay between the ball and the center of the goal
        auto targetPositions = keeperArc.intersectionWithLine(ball->getPos(), field.getOurGoalCenter());

        if (targetPositions.first) {
            return std::make_pair(targetPositions.first.value(), KEEPER);
        } else if (targetPositions.second) {
            return std::make_pair(targetPositions.second.value(), KEEPER);
        }
    }

    // Default position
    return std::make_pair(field.getOurGoalCenter() + Vector2(DISTANCE_FROM_GOAL_FAR, 0), KEEPER);
}

}  // namespace rtt::ai::stp::tactic
