/// Places our robot between the ball and a given target position (either enemy robot or position to defend)

#include "stp/tactics/passive/BlockBall.h"

#include <roboteam_utils/Circle.h>

#include "stp/computations/PositionComputations.h"
#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

BlockBall::BlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> BlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getField() || !skillStpInfo.getBall() || !skillStpInfo.getRobot() || !(skillStpInfo.getEnemyRobot() || skillStpInfo.getPositionToDefend()))
        return std::nullopt;

    auto defendPos = info.getEnemyRobot() ? info.getEnemyRobot().value()->getPos() : info.getPositionToDefend().value();
    auto targetPosition = calculateTargetPosition(info.getBall().value(), defendPos, info.getBlockDistance());

    // Make sure this position is valid
    targetPosition = FieldComputations::projectPointToValidPositionOnLine(info.getField().value(), targetPosition, defendPos, info.getBall()->get()->position);

    skillStpInfo.setPositionToMoveTo(targetPosition);

    auto targetAngle = (info.getBall()->get()->position - info.getRobot()->get()->getPos()).angle();
    skillStpInfo.setAngle(targetAngle);

    return skillStpInfo;
}

bool BlockBall::isEndTactic() noexcept { return true; }

bool BlockBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockBall::shouldTacticReset(const StpInfo &info) noexcept { return false; }

const char *BlockBall::getName() { return "Block Ball"; }

Vector2 BlockBall::calculateTargetPosition(const world::view::BallView &ball, Vector2 defendPos, BlockDistance blockDistance) noexcept {
    auto targetToBall = ball->position - defendPos;

    double distance;
    switch (blockDistance) {
        case BlockDistance::ROBOTRADIUS:
            distance = control_constants::ROBOT_RADIUS;
            break;
        case BlockDistance::CLOSE:
            // Default distance of 0.5m. If the ball is closer than that to the enemy, stand right in front of the ball instead.
            distance = std::min(0.5, targetToBall.length() - control_constants::ROBOT_RADIUS);
            break;
        case BlockDistance::PARTWAY:
            distance = targetToBall.length() * 0.4;
            break;
        case BlockDistance::HALFWAY:
            distance = targetToBall.length() / 2;
            break;
        case BlockDistance::FAR:
            distance = targetToBall.length() - 0.5;
            break;
        default:
            distance = targetToBall.length() / 2;
            break;
    }

    // Do not get closer than 4 robot radii (to avoid collisions)
    distance = std::max(4 * control_constants::ROBOT_RADIUS, distance);

    return defendPos + targetToBall.stretchToLength(distance);
}

}  // namespace rtt::ai::stp::tactic