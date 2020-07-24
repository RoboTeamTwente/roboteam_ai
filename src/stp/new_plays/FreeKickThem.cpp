//
// Created by jordi on 07-05-20.
//

#include "stp/new_plays/FreeKickThem.h"

#include "stp/invariants/game_states/FreeKickThemGameStateInvariant.h"
#include "stp/roles/Defender.h"
#include "stp/roles/Formation.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

FreeKickThem::FreeKickThem() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::FreeKickThemGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::FreeKickThemGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Keeper>("keeper"),         std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"), std::make_unique<role::Defender>("defender_2"),
        std::make_unique<role::Defender>("blocker_0"),  std::make_unique<role::Defender>("blocker_1"),
        std::make_unique<role::Defender>("blocker_2"),  std::make_unique<role::Defender>("blocker_3"),
        std::make_unique<role::Defender>("blocker_4"),  std::make_unique<role::Defender>("blocker_5"),
        std::make_unique<role::Formation>("offender")};
}

uint8_t FreeKickThem::score(world_new::World *world) noexcept { return 100; }

void FreeKickThem::calculateInfoForRoles() noexcept {
    calculateInfoForKeeper();
    calculateInfoForBlockers();
    calculateInfoForDefenders();
    calculateInfoForOffenders();
}

void FreeKickThem::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

void FreeKickThem::calculateInfoForBlockers() noexcept {
    // Factor that is multiplied by the minimal avoid ball distance.
    // We do not block enemy robots within this distance, since they are too close to the ball
    const double AVOID_BALL_DISTANCE_FACTOR = 1.5;

    auto enemyRobots = world->getWorld()->getThem();

    if (enemyRobots.empty()) {
        RTT_ERROR("There are no enemy robots, which are necessary for this play!")
        return;
    }
    // We cannot block enemy robots that are too close to the ball
    enemyRobots.erase(
        std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                       [&](const auto enemyRobot) -> bool { return enemyRobot->getDistanceToBall() <= AVOID_BALL_DISTANCE_FACTOR * control_constants::AVOID_BALL_DISTANCE; }),
        enemyRobots.end());

    for (auto &stpInfo : stpInfos) {
        if (stpInfo.first.find("blocker") != std::string::npos) {
            auto roleName = stpInfo.first;

            if (!enemyRobots.empty()) {
                // If there are enemy robots available, block the closest robot to the ball
                auto enemyToDefend = world->getWorld()->getRobotClosestToPoint(world->getWorld()->getBall().value()->getPos(), enemyRobots);

                enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                                 [&](const auto enemyRobot) -> bool { return enemyToDefend && enemyRobot->getId() == enemyToDefend.value()->getId(); }));

                if (enemyToDefend)
                    stpInfos[roleName].setPositionToDefend(enemyToDefend.value()->getPos());
                else
                    stpInfos[roleName].setPositionToDefend(std::nullopt);
                stpInfos[roleName].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
                stpInfos[roleName].setBlockDistance(BlockDistance::FAR);
            } else {
                // TODO: Improve default behaviour when there are no enemy robots to block
                stpInfos[roleName].setPositionToDefend(field.getOurGoalCenter());
                stpInfos[roleName].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world_new::them));
                stpInfos[roleName].setBlockDistance(BlockDistance::HALFWAY);
            }
        }
    }
}

void FreeKickThem::calculateInfoForDefenders() noexcept {
    stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_0"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["defender_0"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_1"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_1"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_2"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);
}

void FreeKickThem::calculateInfoForOffenders() noexcept { stpInfos["offender"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, 0.0)); }

bool FreeKickThem::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap FreeKickThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"defender_0", {closeToOurGoalFlag}});
    flagMap.insert({"defender_1", {closeToOurGoalFlag}});
    flagMap.insert({"defender_2", {closeToOurGoalFlag}});
    flagMap.insert({"blocker_0", {closeToBallFlag}});
    flagMap.insert({"blocker_1", {closeToBallFlag}});
    flagMap.insert({"blocker_2", {closeToBallFlag}});
    flagMap.insert({"blocker_3", {closeToBallFlag}});
    flagMap.insert({"blocker_4", {closeToBallFlag}});
    flagMap.insert({"blocker_5", {closeToBallFlag}});
    flagMap.insert({"offender", {closeToBallFlag}});

    return flagMap;
}

const char *FreeKickThem::getName() { return "Free Kick Them"; }

}  // namespace rtt::ai::stp::play
