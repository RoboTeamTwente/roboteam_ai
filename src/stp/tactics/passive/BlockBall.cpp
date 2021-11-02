//
// Created by timovdk on 5/12/20.
/// Will position the Robot between the BALL and another Robot

/// PASSIVE
//

#include "stp/tactics/passive/BlockBall.h"

#include <roboteam_utils/Circle.h>

#include "control/ControlUtils.h"
#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

BlockBall::BlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> BlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getField() || !skillStpInfo.getBall() || !skillStpInfo.getRobot() || !(skillStpInfo.getEnemyRobot() || skillStpInfo.getPositionToDefend())) return std::nullopt;

    auto field = info.getField().value();
    auto ball = info.getBall().value();
    auto robot = info.getRobot().value();
    auto defendPos = (info.getEnemyRobot()) ? info.getEnemyRobot().value()->getPos() : info.getPositionToDefend().value();
    auto robotToBall = ball->getPos() - robot->getPos();

    auto targetPosition = calculateTargetPosition(ball, defendPos);

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

Vector2 BlockBall::calculateTargetPosition(const world::view::BallView &ball, Vector2 defendPos) noexcept {
    Vector2 targetPosition{};
    // Stay between the ball and the defend position
    // TODO: Tune this distance
    const double BLOCK_DISTANCE(0.5);
    auto ballCircle = Circle(ball->getPos(), BLOCK_DISTANCE);
    // Project the defend pos on the circle to get the position to block
    targetPosition = ballCircle.project(defendPos);
    targetPosition = ball->getPos().scale(2)-targetPosition;
    return targetPosition;
}

}  // namespace rtt::ai::stp::tactic