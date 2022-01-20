//
// Created by agata on 14/01/2022.
//

#include "stp/plays/defensive/DefendPass.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Harasser.h"

namespace rtt::ai::stp::play {

DefendPass::DefendPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::TheyHaveBall);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::TheyHaveBall);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")), std::make_unique<role::Defender>(role::Defender("defender_1")),
        std::make_unique<role::Defender>(role::Defender("defender_2")), std::make_unique<role::Harasser>(role::Harasser("harassing_defender"))};
}

uint8_t DefendPass::score(PlayEvaluator &playEvaluator) noexcept {
    auto world = playEvaluator.getWorld();
    auto field = world->getField().value();
    auto enemyRobot = world->getWorld()->getRobotClosestToBall(world::them);
    auto position = distanceFromPointToLine(field.getBottomLeftCorner(), field.getTopLeftCorner(), enemyRobot->get()->getPos());
    return 255 * (position / field.getFieldLength());
}

Dealer::FlagMap DefendPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closestToBallFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToBallFlag}}});
    flagMap.insert({"harassing_defender", {DealerFlagPriority::HIGH_PRIORITY, {closestToBallFlag}}});

    return flagMap;
}

void DefendPass::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForKeeper();
    calculateInfoForHarassers();
}

void DefendPass::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                     [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); }),
                      enemyRobots.end());

    auto enemyClosestToOurGoal = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                     [&](const auto enemyRobot) -> bool { return enemyClosestToOurGoal && enemyRobot->getId() == enemyClosestToOurGoal.value()->getId(); }),
                      enemyRobots.end());

    auto remainingEnemy = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    stpInfos["defender_1"].setPositionToDefend(remainingEnemy->get()->getPos());
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(enemyClosestToOurGoal->get()->getPos());
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);
}

void DefendPass::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

void DefendPass::calculateInfoForHarassers() noexcept {
    stpInfos["harassing_defender"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["harassing_defender"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["harassing_defender"].setBlockDistance(BlockDistance::CLOSE);
}

const char *DefendPass::getName() { return "Defend Pass"; }

}  // namespace rtt::ai::stp::play