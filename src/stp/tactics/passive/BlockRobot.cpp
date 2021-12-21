//
// Created by jessevw on 12.03.20.
/// Places a robot between a given TARGET and a given ENEMY at a distance from the the ENEMY
/// TODO-Max de-hardcode the distance (with speed of the enemy? Standstill is block really close?)

/// PASSIVE
//

#include "stp/tactics/passive/BlockRobot.h"

#include <world/FieldComputations.h>

#include "stp/computations/PositionComputations.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"
#include <roboteam_utils/Print.h>

namespace rtt::ai::stp::tactic {

BlockRobot::BlockRobot() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()}; }

// remove roation

std::optional<StpInfo> BlockRobot::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getEnemyRobot() || !skillStpInfo.getPositionToDefend() || !skillStpInfo.getField() || !skillStpInfo.getRobot()) return std::nullopt;

//    // maybe do not need it
//    auto desiredAngle = (info.getEnemyRobot()->get()->getPos() - info.getRobot()->get()->getPos()).angle();
//    skillStpInfo.setAngle(desiredAngle);

    skillStpInfo.setAngle(calculateAngle(info.getEnemyRobot().value(), info.getPositionToDefend().value()));
    RTT_DEBUG("Position to defend: ", info.getPositionToDefend().value());
    RTT_DEBUG("Desired angle: ", calculateAngle(info.getEnemyRobot().value(), info.getPositionToDefend().value()));

    // actual position to defend
    auto defendPos = info.getPositionToDefend().value();
    RTT_DEBUG("Defend pos: ", defendPos);

    auto desiredRobotPosition = calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), defendPos);
    RTT_DEBUG("Robot position first: ", desiredRobotPosition);

    // Minimum distance from the defense area
    auto margin = 2.5 * control_constants::ROBOT_RADIUS;

    // Project the target position outside the defense area if it is in it (within the margin)
    if (FieldComputations::pointIsInDefenseArea(info.getField().value(), desiredRobotPosition, margin)) {
        desiredRobotPosition =
            PositionComputations::ProjectPositionOutsideDefenseAreaOnLine(info.getField().value(), desiredRobotPosition, defendPos, info.getEnemyRobot()->get()->getPos(), margin);
    }

    skillStpInfo.setPositionToMoveTo(desiredRobotPosition);
    RTT_DEBUG("Desired position ", desiredRobotPosition);

    return skillStpInfo;
}

double BlockRobot::calculateAngle(const world::view::RobotView enemy, const Vector2 &targetLocation) {
    Vector2 lineEnemyToTarget = enemy->getPos() - targetLocation;
    return lineEnemyToTarget.angle();
}

Vector2 BlockRobot::calculateDesiredRobotPosition(BlockDistance blockDistance, const world::view::RobotView enemy, const Vector2 &targetLocation) {
//    auto distanceToTarget = enemy->getPos() - targetLocation;

    Vector2 lineEnemyToTarget = targetLocation - enemy->getPos();
    double distance;
    switch (blockDistance) {
        case BlockDistance::CLOSE:
            distance = 0.5;
            break;
        case BlockDistance::HALFWAY:
            distance = lineEnemyToTarget.length() / 2;
            break;
        case BlockDistance::FAR:
            distance = lineEnemyToTarget.length() - 0.5;
            break;
    }

//    if (distance > distanceToTarget.length() || distance < 4 * control_constants::ROBOT_RADIUS) {
//        distance = 4 * control_constants::ROBOT_RADIUS;
//    }
    auto movePosition = lineEnemyToTarget * distance;
    return movePosition + enemy->getPos();
}

bool BlockRobot::isEndTactic() noexcept { return true; }

bool BlockRobot::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockRobot::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

const char *BlockRobot::getName() { return "Block Robot"; }

}  // namespace rtt::ai::stp::tactic
