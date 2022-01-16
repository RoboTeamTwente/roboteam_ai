//
// Created by jordi on 27-03-20.
//

#include "stp/plays/defensive/DefendShotFour.h"

#include "stp/computations/PositionComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/passive/Harasser.h"

namespace rtt::ai::stp::play {

DefendShotFour::DefendShotFour() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::BallCloseToThem);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::BallShotOrCloseToThem);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                       std::make_unique<role::Defender>(role::Defender("defender_1")),
                                                                                       std::make_unique<role::Defender>(role::Defender("defender_2")),
                                                                                       std::make_unique<role::Harasser>(role::Harasser("harasser")),};
}

uint8_t DefendShotFour::score(PlayEvaluator &playEvaluator) noexcept {
    auto enemyRobot = world->getWorld()->getRobotClosestToBall(world::them);
    auto position = distanceFromPointToLine(field.getBottomLeftCorner(), field.getTopLeftCorner(), enemyRobot->get()->getPos());
    return 255 * (field.getFieldLength() - position) / field.getFieldLength();
}

Dealer::FlagMap DefendShotFour::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag closestToBallFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {closestToBallFlag}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::REQUIRED, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::REQUIRED, {closeToOurGoalFlag}}});

    return flagMap;
}

void DefendShotFour::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForHarassers();
    calculateInfoForKeeper();
}

// TODO-Max move to Tactics
void DefendShotFour::calculateInfoForDefenders() noexcept {
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    stpInfos["defender_1"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_1"].setEnemyRobot(enemyClosestToBall);
    stpInfos["defender_1"].setBlockDistance(BlockDistance::FAR);

    stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_2"].setEnemyRobot(enemyClosestToBall);
    stpInfos["defender_2"].setBlockDistance(BlockDistance::FAR);
}

void DefendShotFour::calculateInfoForHarassers() noexcept { stpInfos["harasser"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them)); }

void DefendShotFour::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::us).value()->getPos());
    stpInfos["keeper"].setKickOrChip(KickOrChip::CHIP);
}

const char *DefendShotFour::getName() { return "Defend Shot Four"; }

}  // namespace rtt::ai::stp::play
