//
// Created by jordi on 07-05-20.
//

#include "stp/new_plays/FreeKickThem.h"

#include "stp/invariants/game_states/FreeKickThemGameStateInvariant.h"
#include "stp/new_roles/Keeper.h"
#include "stp/new_roles/Defender.h"
#include "stp/new_roles/Formation.h"

namespace rtt::ai::stp::play {

FreeKickThem::FreeKickThem() : Play() {
    startPlayInvariants.clear();
    //startPlayInvariants.emplace_back(std::make_unique<invariant::FreeKickThemGameStateInvariant>());

    keepPlayInvariants.clear();
    //keepPlayInvariants.emplace_back(std::make_unique<invariant::FreeKickThemGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
            std::make_unique<role::Keeper>(role::Keeper("keeper")),
            std::make_unique<role::Defender>(role::Defender("defender_0")),
            std::make_unique<role::Defender>(role::Defender("defender_1")),
            std::make_unique<role::Defender>(role::Defender("defender_2")),
            std::make_unique<role::Defender>(role::Defender("blocker_0")),
            std::make_unique<role::Defender>(role::Defender("blocker_1")),
            std::make_unique<role::Defender>(role::Defender("blocker_2")),
            std::make_unique<role::Defender>(role::Defender("blocker_3")),
            std::make_unique<role::Defender>(role::Defender("blocker_4")),
            std::make_unique<role::Defender>(role::Defender("blocker_5")),
            std::make_unique<role::Formation>(role::Formation("offender"))};
}

uint8_t FreeKickThem::score(world_new::World* world) noexcept { return 1000; }

void FreeKickThem::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    // Blockers
    const double MINIMAL_DISTANCE_FROM_BALL = 0.5;
    const int NUMBER_OF_BLOCKERS = 6;

    auto enemyRobots = world->getWorld()->getThem();

    // We cannot block enemy robots that are too close to the ball
    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(), [&] (const auto enemyRobot) -> bool {
        return enemyRobot->getDistanceToBall() <= 1.5 * MINIMAL_DISTANCE_FROM_BALL;
    }), enemyRobots.end());

    for (int i = 0; i < NUMBER_OF_BLOCKERS; i++) {
        auto roleName = "blocker_" + std::to_string(i);


        if (enemyRobots.size() != 0) {
            // If there are enemy robots available, block the closest robot to the ball
            auto enemyToDefend = world->getWorld()->getRobotClosestToPoint(
                    world->getWorld()->getBall().value()->getPos(), enemyRobots);

            enemyRobots.erase(
                std::remove_if(enemyRobots.begin(), enemyRobots.end(), [&](const auto enemyRobot) -> bool {
                    return enemyRobot->getId() == enemyToDefend->getId();
                }));

            stpInfos[roleName].setPositionToDefend(enemyToDefend->getPos());
            stpInfos[roleName].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
            stpInfos[roleName].setBlockDistance(FAR);
        } else {
            // TODO: Improve default behaviour when there are no enemy robots to block
            stpInfos[roleName].setPositionToDefend(field.getOurGoalCenter());
            stpInfos[roleName].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world_new::them));
            stpInfos[roleName].setBlockDistance(HALFWAY);
        }
    }

    // Defenders
    stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_0"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["defender_0"].setBlockDistance(HALFWAY);

    stpInfos["defender_1"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_1"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["defender_1"].setBlockDistance(HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_2"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["defender_2"].setBlockDistance(HALFWAY);

    // Offender
    stpInfos["offender"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, 0.0));
}

bool FreeKickThem::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap FreeKickThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"defender_0", {closeToOurGoalFlag}});
    flagMap.insert({"defender_1", {closeToOurGoalFlag}});
    flagMap.insert({"defender_2", {closeToOurGoalFlag}});
    flagMap.insert({"blocker_0", {closeToBallFlag}});
    flagMap.insert({"blocker_1", {closeToBallFlag}});
    flagMap.insert({"blocker_2", {closeToBallFlag}});
    flagMap.insert({"blocker_3", {not_important}});
    flagMap.insert({"blocker_4", {not_important}});
    flagMap.insert({"blocker_5", {not_important}});
    flagMap.insert({"offender", {not_important}});

    return flagMap;
}

const char *FreeKickThem::getName() { return "Free Kick Them"; }

}  // namespace rtt::ai::stp::play