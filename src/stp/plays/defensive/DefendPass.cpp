//
// Created by agata on 14/01/2022.
//

#include "stp/plays/defensive/DefendPass.h"

#include <world/views/RobotView.hpp>

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/passive/Harasser.h"

namespace rtt::ai::stp::play {

DefendPass::DefendPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::TheyHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::TheyHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")),
        std::make_unique<role::Defender>(role::Defender("defender_1")),
        std::make_unique<role::Defender>(role::Defender("defender_2")),
        std::make_unique<role::Defender>(role::Defender("defender_helper_1")),
        std::make_unique<role::Defender>(role::Defender("defender_helper_2")),
        std::make_unique<role::Defender>(role::Defender("midfielder_1")),
        std::make_unique<role::Defender>(role::Defender("midfielder_2")),
        std::make_unique<role::Defender>(role::Defender("midfielder_3")),
        std::make_unique<role::Harasser>(role::Harasser("harasser")),
        std::make_unique<role::Formation>(role::Formation("offender_1")),
        std::make_unique<role::Formation>(role::Formation("offender_2")),
    };
}

uint8_t DefendPass::score(PlayEvaluator &playEvaluator) noexcept {
    auto world = playEvaluator.getWorld();
    auto field = world->getField().value();
    auto enemyRobot = world->getWorld()->getRobotClosestToBall(world::them);
    auto position = distanceFromPointToLine(field.getBottomLeftCorner(), field.getTopLeftCorner(), enemyRobot->get()->getPos());
    auto goalVisibility =
        FieldComputations::getPercentageOfGoalVisibleFromPoint(field, true, enemyRobot->get()->getPos(), world->getWorld().value(), enemyRobot->get()->getId(), false);
    return 255 * (position / field.getFieldLength()) * (100 - goalVisibility) / 100;
}

Dealer::FlagMap DefendPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closestToBallFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"harasser", {DealerFlagPriority::HIGH_PRIORITY, {closestToBallFlag}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_helper_1", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_helper_2", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"offender_1", {DealerFlagPriority::LOW_PRIORITY, {closeToTheirGoalFlag}}});
    flagMap.insert({"offender_2", {DealerFlagPriority::LOW_PRIORITY, {closeToTheirGoalFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_3", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});

    return flagMap;
}

void DefendPass::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForKeeper();
    calculateInfoForHarassers();
    calculateInfoForOffenders();
}

void DefendPass::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    stpInfos["defender_1"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_1"].setBlockDistance(BlockDistance::PARTWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_2"].setBlockDistance(BlockDistance::PARTWAY);

    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                     [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); }),
                      enemyRobots.end());

    auto enemyClosestToOurGoalOne = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                     [&](const auto enemyRobot) -> bool { return enemyClosestToOurGoalOne && enemyRobot->getId() == enemyClosestToOurGoalOne.value()->getId(); }),
                      enemyRobots.end());

    auto enemyClosestToOurGoalTwo = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                     [&](const auto enemyRobot) -> bool { return enemyClosestToOurGoalTwo && enemyRobot->getId() == enemyClosestToOurGoalTwo.value()->getId(); }),
                      enemyRobots.end());

    stpInfos["defender_helper_1"].setPositionToDefend(enemyClosestToOurGoalOne->get()->getPos());
    stpInfos["defender_helper_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_helper_2"].setPositionToDefend(enemyClosestToOurGoalTwo->get()->getPos());
    stpInfos["defender_helper_2"].setBlockDistance(BlockDistance::HALFWAY);

    std::map<double, Vector2> enemyMap;

    for (auto enemy : enemyRobots) {
        double score = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, true, enemy->getPos(), world->getWorld().value(), enemy->getId(), false) *
                       FieldComputations::getDistanceToGoal(field, false, enemy->getPos());
        enemyMap.template insert(std::pair<double, Vector2>(score, enemy->getPos()));
    }

    stpInfos["midfielder_1"].setPositionToDefend(enemyMap.rbegin()->second);
    stpInfos["midfielder_1"].setBlockDistance(BlockDistance::HALFWAY);

    enemyMap.erase(prev(enemyMap.end()));

    stpInfos["midfielder_2"].setPositionToDefend(enemyMap.end()->second);
    stpInfos["midfielder_2"].setBlockDistance(BlockDistance::HALFWAY);

    enemyMap.erase(prev(enemyMap.end()));

    stpInfos["midfielder_3"].setPositionToDefend(enemyMap.rbegin()->second);
    stpInfos["midfielder_3"].setBlockDistance(BlockDistance::HALFWAY);
}

void DefendPass::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
    stpInfos["keeper"].setKickOrChip(KickOrChip::KICK);
}

void DefendPass::calculateInfoForHarassers() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    stpInfos["harasser"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["harasser"].setEnemyRobot(enemyClosestToBall);
    stpInfos["harasser"].setBlockDistance(BlockDistance::CLOSE);
}

void DefendPass::calculateInfoForOffenders() noexcept {
    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();  // maybe put them for the most open position for possible pass

    stpInfos["offender_1"].setPositionToMoveTo(Vector2(length / 4, width / 6));
    stpInfos["offender_2"].setPositionToMoveTo(Vector2(length / 4, -width / 6));
}

const char *DefendPass::getName() { return "Defend Pass"; }

}  // namespace rtt::ai::stp::play