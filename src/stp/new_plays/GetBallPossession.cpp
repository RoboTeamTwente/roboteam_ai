//
// Created by jordi on 11-05-20.
//

#include "stp/invariants/BallClosestToUsInvariant.h"
#include "include/roboteam_ai/stp/new_plays/GetBallPossession.h"

#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "stp/invariants/BallIsFreeInvariant.h"
#include "stp/new_roles/Keeper.h"
#include "stp/new_roles/BallGetter.h"
#include "stp/new_roles/Defender.h"
#include "stp/new_roles/Formation.h"

namespace rtt::ai::stp::play {

GetBallPossession::GetBallPossession() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallIsFreeInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());

    // TODO: Add first arrival to ball invariant

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallIsFreeInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());


    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
            std::make_unique<role::Keeper>(role::Keeper("keeper")),
            std::make_unique<role::BallGetter>(role::BallGetter("ball_getter")),
            std::make_unique<role::Defender>(role::Defender("defender_0")),
            std::make_unique<role::Defender>(role::Defender("defender_1")),
            std::make_unique<role::Defender>(role::Defender("defender_2")),
            std::make_unique<role::Formation>(role::Formation("midfielder_0")),
            std::make_unique<role::Formation>(role::Formation("midfielder_1")),
            std::make_unique<role::Formation>(role::Formation("midfielder_2")),
            std::make_unique<role::Formation>(role::Formation("offender_0")),
            std::make_unique<role::Formation>(role::Formation("offender_1")),
            std::make_unique<role::Formation>(role::Formation("offender_2"))};
}

uint8_t GetBallPossession::score(world_new::World* world) noexcept { return 80; }

void GetBallPossession::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    stpInfos["ball_getter"].setPositionToShootAt(Vector2{0,0});

    stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_0"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world_new::them));
    stpInfos["defender_0"].setBlockDistance(HALFWAY);

    stpInfos["defender_1"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_1"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurBottomGoalSide(), world_new::them));
    stpInfos["defender_1"].setBlockDistance(HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_2"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurTopGoalSide(), world_new::them));
    stpInfos["defender_2"].setBlockDistance(HALFWAY);

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["midfielder_0"].setPositionToMoveTo(Vector2(0.0, width / 4));
    stpInfos["midfielder_1"].setPositionToMoveTo(Vector2(0.0, -width / 4));
    stpInfos["midfielder_2"].setPositionToMoveTo(Vector2(-length / 8, 0.0));

    stpInfos["offender_0"].setPositionToMoveTo(Vector2(length / 4, width / 6));
    stpInfos["offender_1"].setPositionToMoveTo(Vector2(length / 4, -width / 6));
    stpInfos["offender_2"].setPositionToMoveTo(Vector2(length / 4, 0.0));
}

bool GetBallPossession::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap GetBallPossession::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag ballGetterFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"ball_getter", {ballGetterFlag}});
    flagMap.insert({"defender_0", {closeToOurGoalFlag}});
    flagMap.insert({"defender_1", {closeToOurGoalFlag}});
    //flagMap.insert({"defender_2", {closeToOurGoalFlag}});
    //flagMap.insert({"midfielder_0", {not_important}});
    //flagMap.insert({"midfielder_1", {not_important}});
    flagMap.insert({"midfielder_2", {not_important}});
    //flagMap.insert({"offender_0", {closeToTheirGoalFlag}});
/*    flagMap.insert({"offender_1", {closeToTheirGoalFlag}});
    flagMap.insert({"offender_2", {closeToTheirGoalFlag}});*/
    return flagMap;
}

const char* GetBallPossession::getName() { return "Get Ball Possession"; }

}  // namespace rtt::ai::stp::play
