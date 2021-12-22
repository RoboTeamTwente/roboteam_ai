// Created by jessevw on 12.03.20.
/// Places a robot between a given TARGET and a given ENEMY at a distance from the the ENEMY
/// TODO-Max de-hardcode the distance (with speed of the enemy? Standstill is block really close?)

/// PASSIVE
//

#include "stp/tactics/passive/BlockRobot.h"

#include <roboteam_utils/Print.h>
#include <world/FieldComputations.h>

#include "stp/computations/PositionComputations.h"
#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

BlockRobot::BlockRobot() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> BlockRobot::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getEnemyRobot() || !skillStpInfo.getPositionToDefend()) return std::nullopt;

    skillStpInfo.setAngle(calculateAngle(info.getEnemyRobot().value(), info.getPositionToDefend().value()));

    // Minimum distance from the defense area
    auto margin = 2.5 * control_constants::ROBOT_RADIUS;
    auto marginDefense =  2.5* control_constants::ROBOT_RADIUS;

    auto enemyDistanceToDefenseZone =
        FieldComputations::getDistanceToDefenseZone(info.getField().value(), true, info.getEnemyRobot()->get()->getPos(), marginDefense, marginDefense);

    auto desiredRobotPosition =
        calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), info.getPositionToDefend().value(), false, enemyDistanceToDefenseZone);
    RTT_DEBUG("NORMAL ", desiredRobotPosition);

//    // check if the enemy robot is below our penalty line (closer to our goal than the penalty line)
//    if (FieldComputations::isBelowPenaltyLine(info.getField().value(), true, info.getRobot()->get()->getPos(), marginDefense, marginDefense)) {
//        desiredRobotPosition =
//            calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), info.getPositionToDefend().value(), true, enemyDistanceToDefenseZone);
//    }

    // Project the target position outside the defense area if it is in it (within the margin)
    if (FieldComputations::pointIsInDefenseArea(info.getField().value(), desiredRobotPosition, margin)) {
        auto projectedDesiredRobotPosition = PositionComputations::ProjectPositionOutsideDefenseAreaOnLine(
            info.getField().value(), desiredRobotPosition, info.getPositionToDefend().value(), info.getEnemyRobot()->get()->getPos(), margin);
        RTT_DEBUG("PROJECT ", projectedDesiredRobotPosition);
        desiredRobotPosition =
            calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), projectedDesiredRobotPosition, false, enemyDistanceToDefenseZone);
        // check if the enemy robot is below our penalty line (closer to our goal than the penalty line)
        if (FieldComputations::isBelowPenaltyLine(info.getField().value(), true, info.getEnemyRobot()->get()->getPos(), marginDefense, marginDefense)) {
            desiredRobotPosition =
                calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), projectedDesiredRobotPosition, true, enemyDistanceToDefenseZone);
        }
    }

    auto distance = FieldComputations::getDistanceToDefenseZone(info.getField().value(), true, info.getRobot()->get()->getPos(), margin, margin);

    if ((info.getRobot()->get()->getPos() - robotPosBefore).length() <= control_constants::ROBOT_RADIUS && distance <=  2 * margin && (info.getRobot()->get()->getPos() - info.getEnemyRobot()->get()->getPos()).length() <= 3 * margin) {
        withinMarginCount += 1;
    } else {
        withinMarginCount = 0;
    }
    if (waitTick > 30) {
        robotPosBefore = info.getRobot()->get()->getPos();
        waitTick = 0;
    }

    if (withinMarginCount >= 10) {
       desiredRobotPosition = info.getRobot()->get()->getPos();
        RTT_DEBUG("NO MOVEMENT");
    }

    skillStpInfo.setPositionToMoveTo(desiredRobotPosition);
    RTT_DEBUG("RobotPos ", info.getRobot()->get()->getPos());
    RTT_DEBUG("WAIT TICK ", waitTick);
    RTT_DEBUG("MARGIN ", withinMarginCount);

    waitTick += 1;

    return skillStpInfo;
}

double BlockRobot::calculateAngle(const world::view::RobotView enemy, const Vector2 &targetLocation) {
    Vector2 lineEnemyToTarget = enemy->getPos() - targetLocation;
    return lineEnemyToTarget.angle();
}

Vector2 BlockRobot::calculateDesiredRobotPosition(BlockDistance blockDistance, const world::view::RobotView enemy, const Vector2 &targetLocation, bool isBelowPenalty,
                                                  double enemyDistance) {
    auto lineEnemyToTarget = targetLocation - enemy->getPos();
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

    if (enemyDistance < 3* control_constants::ROBOT_RADIUS || distance < 4 * control_constants::ROBOT_RADIUS) {
        distance = 3 * control_constants::ROBOT_RADIUS;
        RTT_DEBUG("CLOSE");
    }
    if (enemyDistance < 2 * control_constants::ROBOT_RADIUS && isBelowPenalty) {
        RTT_DEBUG("HERE", enemy->getPos());
//        if (enemy->getPos().x < targetLocation.x) {
//            return enemy->getPos() + (-distance, distance);
//        } else {
//            return enemy->getPos() + (distance, -distance);
//        }
        auto movePosition = lineEnemyToTarget.stretchToLength(distance);
        return enemy->getPos() - movePosition;
    }

    auto movePosition = lineEnemyToTarget.stretchToLength(distance);
    return movePosition + enemy->getPos();
}

bool BlockRobot::isEndTactic() noexcept { return true; }

bool BlockRobot::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockRobot::shouldTacticReset(const StpInfo &info) noexcept { return false; }

const char *BlockRobot::getName() { return "Block Robot"; }

}  // namespace rtt::ai::stp::tactic
