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

    auto posMargin = 0;

    Vector2 desiredRobotPosition;

    //
    auto enemyDistanceToDefenseZone = 10000;

    // Project the target position outside the defense area if it is in it (within the margin)
    if (FieldComputations::pointIsInDefenseArea(info.getField().value(), info.getPositionToDefend().value(), posMargin, margin)) {

        auto projectedRobotPosition = PositionComputations::ProjectPositionOutsideDefenseAreaOnLine(
            info.getField().value(), info.getPositionToDefend().value(), info.getPositionToDefend().value(), info.getEnemyRobot()->get()->getPos(), posMargin);

        auto enemyDistanceToProjectedPos = distanceFromPointToPoint(projectedRobotPosition, info.getEnemyRobot()->get()->getPos());

        bool isBelowPenalty = false;
        RTT_DEBUG("Position projected ", projectedRobotPosition)

        if (FieldComputations::isBelowPenaltyLine(info.getField().value(), true, info.getEnemyRobot()->get()->getPos(), posMargin, posMargin)) {
            isBelowPenalty = true;
            RTT_DEBUG("Is below penalty")
        }

        desiredRobotPosition =
            calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), projectedRobotPosition, isBelowPenalty, enemyDistanceToProjectedPos);

    } else {
        desiredRobotPosition =
            calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), info.getPositionToDefend().value(), false, enemyDistanceToDefenseZone);
    }

    RTT_DEBUG("Desired robot position ", desiredRobotPosition);

    skillStpInfo.setPositionToMoveTo(desiredRobotPosition);

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
            distance = lineEnemyToTarget.length()-2*control_constants::ROBOT_RADIUS;
            break;
    }

    if (distance < 4 * control_constants::ROBOT_RADIUS || enemyDistance < distance) {
        distance = lineEnemyToTarget.length()/2;
        RTT_DEBUG("To close ", enemyDistance);
    }


    if(enemyDistance < 3 * control_constants::ROBOT_RADIUS){
        distance = 4 * control_constants::ROBOT_RADIUS;
        auto movePosition = lineEnemyToTarget.stretchToLength(distance);
        if(isBelowPenalty){
            RTT_DEBUG("BELOW");
            return Vector2(enemy->getPos().x - movePosition.x, enemy->getPos().y);
        } else {
            RTT_DEBUG("ABOVE")
            return Vector2(enemy->getPos().x, enemy->getPos().y + movePosition.y);
        }
    }

    auto movePosition = lineEnemyToTarget.stretchToLength(distance);
    return movePosition + enemy->getPos();
}

bool BlockRobot::isEndTactic() noexcept { return true; }

bool BlockRobot::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockRobot::shouldTacticReset(const StpInfo &info) noexcept { return false; }

const char *BlockRobot::getName() { return "Block Robot"; }

}  // namespace rtt::ai::stp::tactic
