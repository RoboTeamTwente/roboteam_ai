//
// Created by agata on 14/01/2022.
//

#include "stp/plays/defensive/DefendShot.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/RobotDefender.h"

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

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")),           std::make_unique<role::BallDefender>(role::BallDefender("waller_1")),
        std::make_unique<role::BallDefender>(role::BallDefender("waller_2")),     std::make_unique<role::BallDefender>(role::BallDefender("waller_3")),
        std::make_unique<role::RobotDefender>(role::RobotDefender("defender_1")),   std::make_unique<role::RobotDefender>(role::RobotDefender("defender_2")),
        std::make_unique<role::RobotDefender>(role::RobotDefender("robot_defender")),     std::make_unique<role::BallDefender>(role::BallDefender("midfielder_1")),
        std::make_unique<role::BallDefender>(role::BallDefender("midfielder_2")), std::make_unique<role::BallDefender>(role::BallDefender("midfielder_3")),
        std::make_unique<role::BallDefender>(role::BallDefender("midfielder_4")),
    };
}

uint8_t DefendShot::score(const rtt::world::Field& field) noexcept {
    auto enemyRobot = world->getWorld()->getRobotClosestToBall(world::them);
    auto goalVisibility =
        FieldComputations::getPercentageOfGoalVisibleFromPoint(field, true, enemyRobot->get()->getPos(), world->getWorld().value(), enemyRobot->get()->getId(), false);
    auto position = distanceFromPointToLine(field.getBottomLeftCorner(), field.getTopLeftCorner(), enemyRobot->get()->getPos());
    return 255 * (field.getFieldLength() - position) / field.getFieldLength() * goalVisibility / 100;
}

Dealer::FlagMap DefendShot::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag closestToBallFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"robot_defender", {DealerFlagPriority::HIGH_PRIORITY, {closestToBallFlag}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_3", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_3", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_4", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});

    return flagMap;
}

void DefendShot::calculateInfoForRoles() noexcept {
    calculateInfoForWallers();
    calculateInfoForDefenders();
    calculateInfoForRobotDefenders();
    calculateInfoForKeeper();
}

void DefendShot::calculateInfoForWallers() noexcept {
    stpInfos["waller_1"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["waller_1"].setBlockDistance(BlockDistance::CLOSE);

    stpInfos["waller_2"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["waller_2"].setBlockDistance(BlockDistance::CLOSE);

    stpInfos["waller_3"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["waller_3"].setBlockDistance(BlockDistance::CLOSE);
}

void DefendShot::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();

    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); });

    auto enemyClosestToOurGoalOne = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToOurGoalOne && enemyRobot->getId() == enemyClosestToOurGoalOne.value()->getId(); });

    auto enemyClosestToOurGoalTwo = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToOurGoalTwo && enemyRobot->getId() == enemyClosestToOurGoalTwo.value()->getId(); });

    stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_1"].setEnemyRobot(enemyClosestToOurGoalOne);
    stpInfos["defender_1"].setBlockDistance(BlockDistance::CLOSE);

    stpInfos["defender_2"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_2"].setEnemyRobot(enemyClosestToOurGoalTwo);
    stpInfos["defender_2"].setBlockDistance(BlockDistance::CLOSE);

    std::map<double, Vector2> enemyMap;

    for (auto enemy : enemyRobots) {
        double score = FieldComputations::getDistanceToGoal(field, true, enemy->getPos());
        enemyMap.insert({score, enemy->getPos()});
    }

    for (int i = 1; i <= 4; i++) {
        if (!enemyMap.empty()) {
            stpInfos["midfielder_" + std::to_string(i)].setPositionToDefend(enemyMap.begin()->second);
            stpInfos["midfielder_" + std::to_string(i)].setBlockDistance(BlockDistance::CLOSE);
            enemyMap.erase(enemyMap.begin());
        } else {
            break;
        }
    }
}

void DefendShot::calculateInfoForRobotDefenders() noexcept {
    stpInfos["robot_defender"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["robot_defender"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["robot_defender"].setBlockDistance(BlockDistance::CLOSE);
}

void DefendShot::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::us).value()->getPos());
    stpInfos["keeper"].setKickOrChip(KickOrChip::KICK);
}

const char* DefendShot::getName() { return "Defend Shot"; }

}  // namespace rtt::ai::stp::play
