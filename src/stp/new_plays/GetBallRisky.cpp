//
// Created by timovdk on 5/15/20.
//

#include "stp/new_plays/GetBallRisky.h"

#include <include/roboteam_ai/stp/new_roles/Halt.h>
#include <include/roboteam_ai/stp/new_roles/Harasser.h>
#include <include/roboteam_ai/stp/new_roles/PassReceiver.h>

#include "stp/invariants/BallIsFreeInvariant.h"
#include "stp/invariants/WeHaveMajorityInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "stp/new_roles/BallGetter.h"
#include "stp/new_roles/Keeper.h"

namespace rtt::ai::stp::play {

GetBallRisky::GetBallRisky() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallIsFreeInvariant>());
    //startPlayInvariants.emplace_back(std::make_unique<invariant::WeHaveMajorityInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallIsFreeInvariant>());
    //keepPlayInvariants.emplace_back(std::make_unique<invariant::WeHaveMajorityInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::BallGetter>(role::BallGetter("ball_getter")),
                                                                                 std::make_unique<role::Harasser>(role::Harasser("harasser")),
                                                                                 std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_0")),
                                                                                 std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_1")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_5")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_6")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_7")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_8")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_9")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_10"))};
}

uint8_t GetBallRisky::score(world_new::World* world) noexcept { return 120; }

void GetBallRisky::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["harasser"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["receiver_0"].setPositionToMoveTo(Vector2{-5.0, 2.5});
    stpInfos["receiver_1"].setPositionToMoveTo(Vector2{-5.0, -2.5});

}

bool GetBallRisky::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap GetBallRisky::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeper(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag ball_getter(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {keeper}});
    flagMap.insert({"ball_getter", {ball_getter}});
    flagMap.insert({"harasser", {closeToBallFlag}});
    flagMap.insert({"receiver_0", {not_important}});
    flagMap.insert({"receiver_1", {not_important}});
    flagMap.insert({"halt_5", {not_important}});
    flagMap.insert({"halt_6", {not_important}});
    flagMap.insert({"halt_7", {not_important}});
    flagMap.insert({"halt_8", {not_important}});
    flagMap.insert({"halt_9", {not_important}});
    flagMap.insert({"halt_10", {not_important}});
    return flagMap;
}

const char* GetBallRisky::getName() { return "Get Ball Risky"; }

}  // namespace rtt::ai::stp::play
