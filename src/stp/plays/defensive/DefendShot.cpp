//
// Created by agata on 14/01/2022.
//

#include "stp/plays/defensive/DefendShot.h"

#include <stp/roles/passive/Formation.h>

#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/BallDefender.h"

namespace rtt::ai::stp::play {

DefendShot::DefendShot() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::TheyHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::TheyHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                       std::make_unique<role::Formation>(role::Formation("waller_1")),
                                                                                       std::make_unique<role::Formation>(role::Formation("waller_2")),
                                                                                       std::make_unique<role::Formation>(role::Formation("waller_3")),
                                                                                       std::make_unique<role::Formation>(role::Formation("waller_4")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_1")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_2")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_3")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_4")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_5")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("midfielder_6"))};
}

uint8_t DefendShot::score(const rtt::world::Field& field) noexcept { return 255; }

Dealer::FlagMap DefendShot::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag closestToBallFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_3", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_4", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_3", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_4", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_5", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});

    return flagMap;
}

void DefendShot::calculateInfoForRoles() noexcept {
    calculateInfoForWallers();
    calculateInfoForDefenders();
    calculateInfoForKeeper();
}

void DefendShot::calculateInfoForWallers() noexcept {
    stpInfos["waller_1"].setAngle((world->getWorld()->getBall()->get()->position - field.getOurGoalCenter()).angle());
    stpInfos["waller_2"].setAngle((world->getWorld()->getBall()->get()->position - field.getOurGoalCenter()).angle());
    stpInfos["waller_3"].setAngle((world->getWorld()->getBall()->get()->position - field.getOurGoalCenter()).angle());
    stpInfos["waller_4"].setAngle((world->getWorld()->getBall()->get()->position - field.getOurGoalCenter()).angle());

    stpInfos["waller_1"].setPositionToMoveTo(PositionComputations::getWallPosition(0, 4, field, world));
    stpInfos["waller_2"].setPositionToMoveTo(PositionComputations::getWallPosition(1, 4, field, world));
    stpInfos["waller_3"].setPositionToMoveTo(PositionComputations::getWallPosition(2, 4, field, world));
    stpInfos["waller_4"].setPositionToMoveTo(PositionComputations::getWallPosition(3, 4, field, world));
}

void DefendShot::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();

    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); });

    std::map<double, Vector2> enemyMap;

    for (auto enemy : enemyRobots) {
        double score = FieldComputations::getDistanceToGoal(field, true, enemy->getPos());
        enemyMap.insert({score, enemy->getPos()});
    }

    for (int i = 1; i <= 6; i++) {
        if (!enemyMap.empty()) {
            stpInfos["midfielder_" + std::to_string(i)].setPositionToDefend(enemyMap.begin()->second);
            stpInfos["midfielder_" + std::to_string(i)].setBlockDistance(BlockDistance::ROBOTRADIUS);
            enemyMap.erase(enemyMap.begin());
        } else {
            break;
        }
    }
}

void DefendShot::calculateInfoForHarassers() noexcept {}

void DefendShot::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setKickOrChip(KickOrChip::KICK);
}

const char* DefendShot::getName() { return "Defend Shot"; }

}  // namespace rtt::ai::stp::play
