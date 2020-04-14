//
// Created by jordi on 08-04-20.
//

#include <include/roboteam_ai/control/ControlUtils.h>
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

        auto goalToBall = ball->getPos() - field.getOurGoalCenter();

        auto targetPosition = calculateTargetPosition(ball, field);

        auto targetAngle = goalToBall.angle();

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

    Vector2 BlockBall::calculateTargetPosition(world_new::view::BallView ball, world::Field field) noexcept {
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

        // Stay between the ball and the center of the goal
        auto goalToBall = ball->getPos() - field.getOurGoalCenter();

        auto targetPosition = field.getOurGoalCenter() + goalToBall.stretchToLength(field.getGoalWidth()/2);
        auto targetPositionX = std::clamp(targetPosition.x, field.getLeftmostX() + control_constants::ROBOT_RADIUS,
                                          field.getLeftPenaltyPoint().x - control_constants::ROBOT_RADIUS);
        auto targetPositionY = std::clamp(targetPosition.y, field.getBottomLeftPenaltyStretch().begin.y + control_constants::ROBOT_RADIUS,
                                          field.getTopLeftPenaltyStretch().begin.y - control_constants::ROBOT_RADIUS);
        targetPosition = Vector2(targetPositionX, targetPositionY);

        return targetPosition;
    }

}  // namespace rtt::ai::stp::tactic
