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

Vector2 BlockBall::calculateTargetPosition(const world_new::view::BallView& ball, const world::Field& field,
        const world_new::view::RobotView& enemyRobot) noexcept {
    // Ball is moving
    // Intercept ball when it is moving towards the goal
    if (ball->getVelocity().length() > control_constants::BALL_STILL_VEL) {
        auto targetPosition = control::ControlUtils::twoLineIntersection(ball->getPos(), ball->getPos() + ball->getVelocity()*1000, field.getOurGoalCenter() + Vector2(0.2, field.getGoalWidth()/2), field.getOurGoalCenter() + Vector2(0.2, -field.getGoalWidth()/2));
        if (targetPosition.x == -5.8 && targetPosition.y >= -field.getGoalWidth()/2 && targetPosition.y <= field.getGoalWidth()/2) {
            return targetPosition;
        }
    }

    // Opponent is close to ball
    // Block the ball by staying on the shot line of the opponent
    if (enemyRobot->getDistanceToBall() < 1.0) {
        auto start = enemyRobot->getPos();
        auto enemyToBall = ball->getPos() - start;
        auto end = start + enemyToBall.stretchToLength(3.0);
        const auto& startGoal = field.getOurTopGoalSide();
        const auto& endGoal = field.getOurBottomGoalSide();

        if (control::ControlUtils::lineSegmentsIntersect(start, end, startGoal, endGoal)) {
            auto goalPos = control::ControlUtils::twoLineIntersection(start, end, startGoal, endGoal);
            return goalPos + Vector2(field.getGoalWidth()/1.5, 0).rotate(enemyToBall.angle() + M_PI);
        }
    }

    // Stay between the ball and the center of the goal
    auto goalToBall = ball->getPos() - field.getOurGoalCenter();

    auto targetPosition = field.getOurGoalCenter() + goalToBall.stretchToLength(field.getGoalWidth()/1.5);
    auto targetPositionX = std::clamp(targetPosition.x, field.getLeftmostX() + control_constants::ROBOT_RADIUS,
                                      field.getLeftPenaltyPoint().x - control_constants::ROBOT_RADIUS);
    auto targetPositionY = std::clamp(targetPosition.y, field.getBottomLeftPenaltyStretch().begin.y + control_constants::ROBOT_RADIUS,
                                      field.getTopLeftPenaltyStretch().begin.y - control_constants::ROBOT_RADIUS);
    targetPosition = Vector2(targetPositionX, targetPositionY);

    return targetPosition;
}

}  // namespace rtt::ai::stp::tactic
