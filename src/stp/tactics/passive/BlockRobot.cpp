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

    // Distance of the enemy robot to our defense zone
    auto enemyDistanceToDefenseZone =
        FieldComputations::getDistanceToDefenseZone(info.getField().value(), true, info.getEnemyRobot()->get()->getPos(), margin, margin);

    auto desiredRobotPosition =
        calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), info.getPositionToDefend().value(), false, enemyDistanceToDefenseZone);

    // Project the target position outside the defense area if it is in it (within the margin)
    if (FieldComputations::pointIsInDefenseArea(info.getField().value(), desiredRobotPosition, margin)) {

        auto projectedDesiredRobotPosition = PositionComputations::ProjectPositionOutsideDefenseAreaOnLine(
            info.getField().value(), desiredRobotPosition, info.getPositionToDefend().value(), info.getEnemyRobot()->get()->getPos(), margin);

        bool isBelowPenalty = false;
        RTT_DEBUG("Position projected")

        // check if the enemy robot is below our penalty line (closer to our goal than the penalty line)
        if (enemyDistanceToDefenseZone < FieldComputations::isBelowPenaltyLine(info.getField().value(), true, info.getEnemyRobot()->get()->getPos(), margin, margin)) {
            isBelowPenalty = true;
            RTT_DEBUG("Is below penalty")
        }

        desiredRobotPosition =
            calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), projectedDesiredRobotPosition, isBelowPenalty, enemyDistanceToDefenseZone);
    }

    RTT_DEBUG("Desired robot psoition ", desiredRobotPosition);

//    // distance of the blocking robot from our defense zone
//    auto distance = FieldComputations::getDistanceToDefenseZone(info.getField().value(), true, info.getRobot()->get()->getPos(), margin, margin);
//
//    // function based on ticks to handle situation very close to our defense zone, in which robot cannot move closer to the target position
//    if ((info.getRobot()->get()->getPos() - robotPosBefore).length() <= control_constants::ROBOT_RADIUS / 4 && distance <= 2 * margin &&
//        (info.getRobot()->get()->getPos() - info.getEnemyRobot()->get()->getPos()).length() <= 2 * margin) {
//        withinMarginCount += 1;
//    } else {
//        withinMarginCount = 0;
//    }
//
//    // update the previous position of the robot after 10 tick so that if the robots move the difference between current position and previous one will be notable
//    if (waitTick > 10) {
//        robotPosBefore = info.getRobot()->get()->getPos();
//        waitTick = 0;
//    }
//
//    // return current robot position as desired position to finish the tactic
//    if (withinMarginCount >= 10) {
//        desiredRobotPosition = info.getRobot()->get()->getPos();
//    }

    skillStpInfo.setPositionToMoveTo(desiredRobotPosition);

//    waitTick += 1;

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
//|| enemyDistance < 3 * control_constants::ROBOT_RADIUS
    if (distance < 4 * control_constants::ROBOT_RADIUS ) {
        distance = 4 * control_constants::ROBOT_RADIUS;
        RTT_DEBUG("To close", enemyDistance);
    }

    // check if enemy is too close to the defense zone and under the penalty line
    if (enemyDistance < 2 * control_constants::ROBOT_RADIUS && isBelowPenalty) {
        RTT_DEBUG("HERE", enemy->getPos());
        auto movePosition = lineEnemyToTarget.stretchToLength(distance);
        auto finalPos = Vector2(enemy->getPos().x - movePosition.x, enemy->getPos().y);
        RTT_DEBUG("MOVE" ,enemy->getPos() - movePosition, "   ", movePosition, "   ", finalPos)
        return finalPos;
    }

    auto movePosition = lineEnemyToTarget.stretchToLength(distance);
    return movePosition + enemy->getPos();
}

bool BlockRobot::isEndTactic() noexcept { return true; }

bool BlockRobot::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockRobot::shouldTacticReset(const StpInfo &info) noexcept { return false; }

const char *BlockRobot::getName() { return "Block Robot"; }

}  // namespace rtt::ai::stp::tactic
