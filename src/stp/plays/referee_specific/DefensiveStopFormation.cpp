//
// Created by agata on 14/01/2022.
//

#include <stp/plays/referee_specific/DefensiveStopFormation.h>

#include "stp/roles/passive/Formation.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/BallDefender.h"

namespace rtt::ai::stp::play {

DefensiveStopFormation::DefensiveStopFormation() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::StopGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::TheyHaveFreeKickNext);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::StopGameState);

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
                                                                                       std::make_unique<role::Formation>("ball_blocker")};
}

uint8_t DefensiveStopFormation::score(const rtt::world::Field& field) noexcept { return 255; }

Dealer::FlagMap DefensiveStopFormation::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag closestToBallFlag(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_3", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"waller_4", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_3", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_4", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_5", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"ball_blocker", {DealerFlagPriority::HIGH_PRIORITY, {notImportant}}});

    return flagMap;
}

void DefensiveStopFormation::calculateInfoForRoles() noexcept {
    calculateInfoForWallers();
    calculateInfoForDefenders();
    calculateInfoForKeeper();
    calculateInfoForBlocker();
}

void DefensiveStopFormation::calculateInfoForWallers() noexcept {
    for (int i = 1; i <= 4; ++i) {
        // For each waller, stand in the right wall position and look at the ball
        auto positionToMoveTo = PositionComputations::getWallPosition(i - 1, 4, field, world);
        stpInfos["waller_" + std::to_string(i)].setPositionToMoveTo(positionToMoveTo);
        stpInfos["waller_" + std::to_string(i)].setAngle((world->getWorld()->getBall()->get()->position - field.getOurGoalCenter()).angle());

        // If the waller is close to its target, ignore collisions
        constexpr double IGNORE_COLLISIONS_DISTANCE = 1.0;
        if (stpInfos["waller_" + std::to_string(i)].getRobot()) {
            if ((stpInfos["waller_" + std::to_string(i)].getRobot()->get()->getPos() - positionToMoveTo).length() < IGNORE_COLLISIONS_DISTANCE)
                stpInfos["waller_" + std::to_string(i)].setShouldAvoidOurRobots(false);
        }
    }
}

void DefensiveStopFormation::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();

    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    erase_if(enemyRobots, [&](const auto enemyRobot) -> bool { return enemyClosestToBall && enemyRobot->getId() == enemyClosestToBall.value()->getId(); });

    std::map<double, Vector2> enemyMap;

    for (auto enemy : enemyRobots) {
        double score = FieldComputations::getDistanceToGoal(field, true, enemy->getPos());
        enemyMap.insert({score, enemy->getPos()});
    }

    for (int i = 1; i <= 5; i++) {
        if (!enemyMap.empty()) {
            stpInfos["midfielder_" + std::to_string(i)].setPositionToDefend(enemyMap.begin()->second);
            stpInfos["midfielder_" + std::to_string(i)].setBlockDistance(BlockDistance::ROBOTRADIUS);
            enemyMap.erase(enemyMap.begin());
        } else {
            break;
        }
    }
}

void DefensiveStopFormation::calculateInfoForBlocker() noexcept { stpInfos["ball_blocker"].setPositionToMoveTo(PositionComputations::getBallBlockPosition(field, world)); }

void DefensiveStopFormation::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setKickOrChip(KickOrChip::KICK);
}

const char* DefensiveStopFormation::getName() { return "Defend Shot"; }

}  // namespace rtt::ai::stp::play
