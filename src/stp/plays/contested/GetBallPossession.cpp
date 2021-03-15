//
// Created by jordi on 11-05-20.
//

#include <stp/roles/passive/Waller.h>
#include "include/roboteam_ai/stp/plays/contested/GetBallPossession.h"
#include "include/roboteam_ai/stp/computations/PositionComputations.h"
#include "stp/invariants/BallClosestToUsInvariant.h"
#include "stp/invariants/BallIsFreeInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "include/roboteam_ai/stp/roles/active/BallGetter.h"
#include "include/roboteam_ai/stp/roles/passive/Defender.h"
#include "include/roboteam_ai/stp/roles/passive/Formation.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

GetBallPossession::GetBallPossession() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallIsFreeInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallIsFreeInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::BallGetter>(role::BallGetter("ball_getter")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_0")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_1")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_0")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                                                                                 std::make_unique<role::Waller>(role::Waller("waller_0")),
                                                                                 std::make_unique<role::Waller>(role::Waller("waller_1")),
                                                                                 std::make_unique<role::Waller>(role::Waller("waller_2"))};
}

uint8_t GetBallPossession::score(world::World* world) noexcept { return 10; }

void GetBallPossession::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    //TODO-Jaro: Find out why GetBallPossession has a shootPos, and remove/improve if necessary
    stpInfos["ball_getter"].setPositionToShootAt(field.getTheirGoalCenter());

    stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_0"].setEnemyRobot(
            world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::them));
    stpInfos["defender_0"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_1"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_1"].setEnemyRobot(
            world->getWorld()->getRobotClosestToPoint(field.getOurBottomGoalSide(), world::them));
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_2"].setEnemyRobot(
            world->getWorld()->getRobotClosestToPoint(field.getOurTopGoalSide(), world::them));
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["midfielder_0"].setPositionToMoveTo(Vector2(0.0, width / 4));
    stpInfos["midfielder_1"].setPositionToMoveTo(Vector2(0.0, -width / 4));
    stpInfos["midfielder_2"].setPositionToMoveTo(Vector2(-length / 8, 0.0));

    int amountDefenders = 3;
    std::vector<Vector2> wallPositions = {};
    if(FieldComputations::pointIsValidPosition(field, world->getWorld().value().getBall().value()->getPos()))
        wallPositions = computations::PositionComputations::determineWallPositions(field,world,amountDefenders);
    if (!wallPositions.empty()) {
        stpInfos["waller_0"].setPositionToMoveTo(wallPositions.at(0));
        stpInfos["waller_1"].setPositionToMoveTo(wallPositions.at(1));
        stpInfos["waller_2"].setPositionToMoveTo(wallPositions.at(2));
    } else {
        stpInfos["waller_0"].setPositionToMoveTo(Vector2(0,0));
        stpInfos["waller_1"].setPositionToMoveTo(Vector2(0,0.2));
        stpInfos["waller_2"].setPositionToMoveTo(Vector2(0,-0.2));
    }
}

bool GetBallPossession::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap GetBallPossession::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag ballGetterFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"ball_getter", {DealerFlagPriority::REQUIRED, {ballGetterFlag}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"midfielder_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"offender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"offender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"offender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

const char* GetBallPossession::getName() { return "Get Ball Possession"; }

}  // namespace rtt::ai::stp::play
