//
// Created by jordi on 08-04-20.
//

#include "control/ControlUtils.h"
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

    auto field = info.getField().value();
    auto ball = info.getBall().value();
    auto keeper = info.getRobot().value();
    auto enemyRobot = info.getEnemyRobot().value();

    auto keeperToBall = ball->getPos() - keeper->getPos();

    auto targetPosition = calculateTargetPosition(ball, field, enemyRobot);

    auto targetAngle = keeperToBall.angle();

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

Vector2 BlockBall::calculateTargetPosition(const world_new::view::BallView &ball, const world::Field &field,
        const world_new::view::RobotView &enemyRobot) noexcept {
    const double DISTANCE_FROM_GOAL_FAR = field.getGoalWidth() / 1.5;
    const double DISTANCE_FROM_GOAL_CLOSE = 2 * control_constants::ROBOT_RADIUS;
    const double ENEMY_CLOSE_TO_BALL_DISTANCE = 1.0;

    // Ball is on our side
    if (ball->getPos().x < 0) {
        auto keeperArc = Arc(field.getOurGoalCenter(), DISTANCE_FROM_GOAL_FAR, -M_PI / 2, M_PI / 2);

        // Ball is moving
        // Intercept ball when it is moving towards the goal
        if (ball->getVelocity().length() > control_constants::BALL_STILL_VEL) {
            auto start = ball->getPos();
            auto end = start + ball->getVelocity().stretchToLength(field.getFieldLength() * 0.5);
            auto startGoal = field.getOurTopGoalSide() + Vector2(DISTANCE_FROM_GOAL_CLOSE, 0);
            auto endGoal = field.getOurBottomGoalSide() + Vector2(DISTANCE_FROM_GOAL_CLOSE, 0);

            if (control::ControlUtils::lineSegmentsIntersect(start, end, startGoal, endGoal)) {
                return control::ControlUtils::twoLineIntersection(start, end, startGoal, endGoal);
            }
        }

        // Opponent is close to ball
        // Block the ball by staying on the shot line of the opponent
        if (enemyRobot->getDistanceToBall() < ENEMY_CLOSE_TO_BALL_DISTANCE) {
            auto start = enemyRobot->getPos();
            auto enemyToBall = ball->getPos() - start;
            auto end = start + enemyToBall.stretchToLength(field.getFieldLength() * 0.5);
            const auto &startGoal = field.getOurTopGoalSide();
            const auto &endGoal = field.getOurBottomGoalSide();

            if (control::ControlUtils::lineSegmentsIntersect(start, end, startGoal, endGoal)) {
                auto goalPos = control::ControlUtils::twoLineIntersection(start, end, startGoal, endGoal);

                auto targetPositions = keeperArc.intersectionWithLine(start, goalPos);

                if (targetPositions.first) {
                    return targetPositions.first.value();
                } else if (targetPositions.second) {
                    return targetPositions.second.value();
                }
            }
        }

        // Stay between the ball and the center of the goal
        auto targetPositions = keeperArc.intersectionWithLine(ball->getPos(), field.getOurGoalCenter());

        if (targetPositions.first) {
            return targetPositions.first.value();
        } else if (targetPositions.second) {
            return targetPositions.second.value();
        }
    }

    // Default position
    return field.getOurGoalCenter() + Vector2(DISTANCE_FROM_GOAL_FAR, 0);
}

}  // namespace rtt::ai::stp::tactic
