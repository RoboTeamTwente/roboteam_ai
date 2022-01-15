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
        std::make_unique<role::Keeper>(role::Keeper("keeper")),         std::make_unique<role::Defender>(role::Defender("defender_1")),
        std::make_unique<role::Defender>(role::Defender("defender_2")), std::make_unique<role::Defender>(role::Defender("blocker_1")),
        std::make_unique<role::Defender>(role::Defender("blocker_2")),  std::make_unique<role::Defender>(role::Defender("blocker_3")),
        std::make_unique<role::Defender>(role::Defender("blocker_4")),  std::make_unique<role::Defender>(role::Defender("blocker_5")),
        std::make_unique<role::Harasser>(role::Harasser("harasser")),   std::make_unique<role::Defender>(role::Defender("offender_1")),
        std::make_unique<role::Defender>(role::Defender("offender_2"))};
}

uint8_t DefendPassFour::score(PlayEvaluator &playEvaluator) noexcept { return 100; }

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
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);
    auto enemyClosestToOurGoal = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::them);

    if (enemyClosestToBall->get()->getId() == enemyClosestToOurGoal->get()->getId()) {
        RTT_DEBUG("THE same");
        stpInfos["defender_1"].setPositionToDefend(field.getOurTopGoalSide());
        stpInfos["defender_1"].setEnemyRobot(enemyClosestToBall);
        stpInfos["defender_1"].setBlockDistance(BlockDistance::FAR);

        stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
        stpInfos["defender_2"].setEnemyRobot(enemyClosestToBall);
        stpInfos["defender_2"].setBlockDistance(BlockDistance::FAR);
    } else {
        RTT_DEBUG("DIFFERENT");
        stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
        stpInfos["defender_1"].setEnemyRobot(enemyClosestToBall);
        stpInfos["defender_1"].setBlockDistance(BlockDistance::FAR);

        stpInfos["defender_2"].setPositionToDefend(enemyClosestToOurGoal->get()->getPos());
        stpInfos["defender_2"].setEnemyRobot(enemyClosestToBall);
        stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);
    }

}

void DefendPassFour::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

void DefendPassFour::calculateInfoForHarassers() noexcept { stpInfos["harasser"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them)); }

const char *DefendPassFour::getName() { return "Defend Pass Four"; }

}  // namespace rtt::ai::stp::play