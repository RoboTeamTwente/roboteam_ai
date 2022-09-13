// Created by jessevw on 12.03.20.
/// Places a robot between a given TARGET and a given ENEMY at a distance from the the ENEMY
/// PASSIVE

#include "stp/tactics/passive/BlockRobot.h"
#include "stp/computations/PositionComputations.h"
#include "stp/skills/GoToPos.h"

namespace rtt::ai::stp::tactic {

BlockRobot::BlockRobot() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> BlockRobot::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getEnemyRobot() || !skillStpInfo.getPositionToDefend()) return std::nullopt;

    skillStpInfo.setAngle(calculateAngle(info.getEnemyRobot().value(), info.getPositionToDefend().value()));

    auto positionToDefend = info.getPositionToDefend().value();

    auto enemyDistanceToPoint = positionToDefend.dist(info.getEnemyRobot()->get()->getPos());

    auto desiredRobotPosition = calculateDesiredRobotPosition(info.getBlockDistance(), info.getEnemyRobot().value(), positionToDefend, enemyDistanceToPoint);

    auto projectedPosition =
        FieldComputations::projectPointToValidPositionOnLine(info.getField().value(), desiredRobotPosition, positionToDefend, info.getEnemyRobot()->get()->getPos());

    skillStpInfo.setPositionToMoveTo(projectedPosition);

    return skillStpInfo;
}

double BlockRobot::calculateAngle(const world::view::RobotView enemy, const Vector2 &targetLocation) {
    Vector2 lineEnemyToTarget = enemy->getPos() - targetLocation;
    return lineEnemyToTarget.angle();
}

/// TODO: fine tune distance
Vector2 BlockRobot::calculateDesiredRobotPosition(BlockDistance blockDistance, const world::view::RobotView enemy, const Vector2 &targetLocation, double enemyDistance) {
    auto lineEnemyToTarget = targetLocation - enemy->getPos(); // predicted trajectory of enemy robot to target
    double distance; // distance from the enemy robot to blocking position

    switch (blockDistance) {
        case BlockDistance::CLOSE:
            distance = 3 * control_constants::ROBOT_RADIUS + enemy->getVel().length(); // get as close to the enemy robot as possible without colliding
            break;
        case BlockDistance::HALFWAY:
            distance = lineEnemyToTarget.length() / 2 + 0.5 * enemy->getVel().length(); // get in the middle of enemy robot and target location
            break;
        case BlockDistance::FAR:
            distance = lineEnemyToTarget.length() - 4 * control_constants::ROBOT_RADIUS; // get close to the target location
            break;
        default:
            distance = lineEnemyToTarget.length() / 2 + 0.5 * enemy->getVel().length(); // default BlockDistance is HALFWAY
            break;
    }

    if (distance < 4 * control_constants::ROBOT_RADIUS || enemyDistance < distance) { // if enemy is closer to target position
        distance = lineEnemyToTarget.length() / 2;
    }

    auto movePosition = lineEnemyToTarget.stretchToLength(distance);
    return movePosition + enemy->getPos();
}

bool BlockRobot::isEndTactic() noexcept { return true; }

bool BlockRobot::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockRobot::shouldTacticReset(const StpInfo &info) noexcept { return false; }

const char *BlockRobot::getName() { return "Block Robot"; }

}  // namespace rtt::ai::stp::tactic
