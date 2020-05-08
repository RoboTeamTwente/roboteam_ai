//
// Created by jordi on 07-05-20.
//

#include "stp/new_plays/FreeKickThem.h"
#include "stp/new_roles/Keeper.h"
#include "stp/new_roles/Defender.h"
#include "stp/invariants/game_states/FreeKickThemGameStateInvariant.h"

namespace rtt::ai::stp::play {

FreeKickThem::FreeKickThem() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::FreeKickThemGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::FreeKickThemGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
            std::make_unique<role::Keeper>(role::Keeper("keeper")), std::make_unique<role::Defender>(role::Defender("defender_0")),
            std::make_unique<role::Defender>(role::Defender("defender_1")), std::make_unique<role::Defender>(role::Defender("defender_2")),
            std::make_unique<role::Defender>(role::Defender("defender_3")), std::make_unique<role::Defender>(role::Defender("defender_4")),
            std::make_unique<role::Defender>(role::Defender("defender_5")), std::make_unique<role::Defender>(role::Defender("defender_6")),
            std::make_unique<role::Defender>(role::Defender("defender_7")), std::make_unique<role::Defender>(role::Defender("defender_8")),
            std::make_unique<role::Defender>(role::Defender("defender_9"))};
}

uint8_t FreeKickThem::score(world_new::World* world) noexcept { return 100; }

void FreeKickThem::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    // Defenders
    const double MINIMAL_DISTANCE_FROM_BALL = 0.5;
    const int NUMBER_OF_DEFENDERS = 10;

    auto enemyRobots = world->getWorld()->getThem();

    // We cannot block enemy robots that are too close to the ball
    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(), [&] (const auto enemyRobot) -> bool {
        return enemyRobot->getDistanceToBall() <= 1.5 * MINIMAL_DISTANCE_FROM_BALL;
    }), enemyRobots.end());

    for (int i = 0; i < NUMBER_OF_DEFENDERS; i++) {
        auto roleName = "defender_" + std::to_string(i);


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
}

bool FreeKickThem::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap FreeKickThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {}});
    flagMap.insert({"defender_0", {}});
    flagMap.insert({"defender_1", {}});
    flagMap.insert({"defender_2", {}});
    flagMap.insert({"defender_3", {}});
    flagMap.insert({"defender_4", {}});
    flagMap.insert({"defender_5", {}});
    flagMap.insert({"defender_6", {}});
    flagMap.insert({"defender_7", {}});
    flagMap.insert({"defender_8", {}});
    flagMap.insert({"defender_9", {}});

    return flagMap;
}

const char *FreeKickThem::getName() {
    return "Free Kick Them";
}

}  // namespace rtt::ai::stp::play