//
// Created by agata on 14/01/2022.
//

#include "stp/plays/defensive/DefendPassFour.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Harasser.h"

namespace rtt::ai::stp::play {

DefendPassFour::DefendPassFour() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::BallCloseToThem);
    startPlayEvaluation.emplace_back(eval::BallOnOurSide);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::BallShotOrCloseToThem);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")), std::make_unique<role::Defender>(role::Defender("defender_1")),
        std::make_unique<role::Defender>(role::Defender("defender_2")), std::make_unique<role::Harasser>(role::Harasser("harasser"))};
}

uint8_t DefendPassFour::score(PlayEvaluator &playEvaluator) noexcept {
    auto enemyRobot = world->getWorld()->getRobotClosestToBall(world::them);
    auto position = distanceFromPointToLine(field.getBottomLeftCorner(), field.getTopLeftCorner(), enemyRobot->get()->getPos());
    return 255 * (position/field.getFieldLength());
}

Dealer::FlagMap DefendPassFour::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closestToBallFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::REQUIRED, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {closestToBallFlag}}});

    return flagMap;
}

void DefendPassFour::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForKeeper();
    calculateInfoForHarassers();
}

void DefendPassFour::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(), [&](const auto enemyRobot) -> bool {
        return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId();
    }));

    auto enemyClosestToOurGoal = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                     [&](const auto enemyRobot) -> bool { return enemyClosestToOurGoal && enemyRobot->getId() == enemyClosestToOurGoal.value()->getId(); }));

    auto remainingEnemy = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    stpInfos["defender_1"].setPositionToDefend(remainingEnemy->get()->getPos());
    stpInfos["defender_1"].setEnemyRobot(enemyClosestToBall);
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(enemyClosestToOurGoal->get()->getPos());
    stpInfos["defender_2"].setEnemyRobot(enemyClosestToBall);
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);
}

void DefendPassFour::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

void DefendPassFour::calculateInfoForHarassers() noexcept { stpInfos["harasser"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them)); }

const char *DefendPassFour::getName() { return "Defend Pass Four"; }

}  // namespace rtt::ai::stp::play