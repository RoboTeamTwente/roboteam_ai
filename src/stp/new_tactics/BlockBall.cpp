//
// Created by jordi on 08-04-20.
//

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
    auto goalToBall = info.getBall().value()->getPos() - field.getOurGoalCenter();

    // Stay between the ball and the center of the goal
    auto targetPosition = field.getOurGoalCenter() + goalToBall.stretchToLength(field.getGoalWidth() / 2.0);
    auto targetPositionX = std::clamp(targetPosition.x, field.getLeftmostX() + control_constants::ROBOT_RADIUS,
            field.getLeftPenaltyPoint().x - control_constants::ROBOT_RADIUS);
    auto targetPositionY = std::clamp(targetPosition.y, field.getBottomLeftPenaltyStretch().begin.y + control_constants::ROBOT_RADIUS,
            field.getTopLeftPenaltyStretch().begin.y - control_constants::ROBOT_RADIUS);
    targetPosition = Vector2(targetPositionX, targetPositionY);

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

}  // namespace rtt::ai::stp::tactic
