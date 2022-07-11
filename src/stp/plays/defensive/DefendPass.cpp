//
// Created by agata on 14/01/2022.
//

#include "stp/plays/defensive/DefendPass.h"

#include <world/views/RobotView.hpp>

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/passive/RobotDefender.h"

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
        std::make_unique<role::BallDefender>(role::BallDefender("defender_1")),
        std::make_unique<role::BallDefender>(role::BallDefender("defender_2")),
        std::make_unique<role::BallDefender>(role::BallDefender("defender_helper_1")),
        std::make_unique<role::BallDefender>(role::BallDefender("defender_helper_2")),
        std::make_unique<role::BallDefender>(role::BallDefender("midfielder_1")),
        std::make_unique<role::BallDefender>(role::BallDefender("midfielder_2")),
        std::make_unique<role::BallDefender>(role::BallDefender("midfielder_3")),
        std::make_unique<role::RobotDefender>(role::RobotDefender("robot_defender")),
        std::make_unique<role::Formation>(role::Formation("offender_1")),
        std::make_unique<role::Formation>(role::Formation("offender_2")),
    };
}

uint8_t DefendPass::score(const rtt::world::Field& field) noexcept {
    auto enemyRobot = world->getWorld()->getRobotClosestToBall(world::them);
    auto position = distanceFromPointToLine(field.getBottomLeftCorner(), field.getTopLeftCorner(), enemyRobot->get()->getPos());
    auto goalVisibility =
        FieldComputations::getPercentageOfGoalVisibleFromPoint(field, true, enemyRobot->get()->getPos(), world->getWorld().value(), enemyRobot->get()->getId(), false);
    return 255 * (position / field.getFieldLength()) * (100 - goalVisibility) / 100;
}

Dealer::FlagMap DefendPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closestToBallFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"robot_defender", {DealerFlagPriority::HIGH_PRIORITY, {closestToBallFlag}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_helper_1", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_helper_2", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_3", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"offender_1", {DealerFlagPriority::LOW_PRIORITY, {closeToTheirGoalFlag}}});
    flagMap.insert({"offender_2", {DealerFlagPriority::LOW_PRIORITY, {closeToTheirGoalFlag}}});

    return flagMap;
}

void DefendPass::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForKeeper();
    calculateInfoForRobotDefenders();
    calculateInfoForOffenders();
}

void DefendPass::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    stpInfos["defender_1"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_1"].setBlockDistance(BlockDistance::PARTWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_2"].setBlockDistance(BlockDistance::PARTWAY);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); });

    auto enemyClosestToOurGoalOne = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToOurGoalOne && enemyRobot->getId() == enemyClosestToOurGoalOne.value()->getId(); });

    auto enemyClosestToOurGoalTwo = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToOurGoalTwo && enemyRobot->getId() == enemyClosestToOurGoalTwo.value()->getId(); });

    stpInfos["defender_helper_1"].setPositionToDefend(enemyClosestToOurGoalOne->get()->getPos());
    stpInfos["defender_helper_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_helper_2"].setPositionToDefend(enemyClosestToOurGoalTwo->get()->getPos());
    stpInfos["defender_helper_2"].setBlockDistance(BlockDistance::HALFWAY);

    std::map<double, Vector2> enemyMap;

    // TODO: figure out better scoring
    for (auto enemy : enemyRobots) {
        double score = FieldComputations::getTotalGoalAngle(field, true, enemy->getPos());
        enemyMap.insert({score, enemy->getPos()});
    }

    for (int i = 1; i <= 3; i++) {
        if (!enemyMap.empty()) {
            stpInfos["midfielder_" + std::to_string(i)].setPositionToDefend(enemyMap.rbegin()->second);
            stpInfos["midfielder_" + std::to_string(i)].setBlockDistance(BlockDistance::HALFWAY);
            enemyMap.erase(prev(enemyMap.end()));
        } else {
            break;
        }
    }
}

void DefendPass::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
    stpInfos["keeper"].setKickOrChip(KickOrChip::KICK);
}

void DefendPass::calculateInfoForRobotDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    stpInfos["robot_defender"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["robot_defender"].setEnemyRobot(enemyClosestToBall);
    stpInfos["robot_defender"].setBlockDistance(BlockDistance::CLOSE);
}

void DefendPass::calculateInfoForOffenders() noexcept {
    stpInfos["offender_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontMidGrid(), gen::OffensivePosition, field, world));
    if (world->getWorld()->getBall().value()->position.y > 0) {
        stpInfos["offender_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
    } else {
        stpInfos["offender_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
    }
}

const char* DefendPass::getName() { return "Defend Pass"; }

}  // namespace rtt::ai::stp::play