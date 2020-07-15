//
// Created by timo on 3/30/20.
//

#include "stp/new_plays/AggressiveFormation.h"

#include "stp/invariants/game_states/StopGameStateInvariant.h"
#include "stp/new_roles/BallAvoider.h"

namespace rtt::ai::stp::play {

    AggressiveFormation::AggressiveFormation() : Play() {
        startPlayInvariants.clear();
        startPlayInvariants.emplace_back(std::make_unique<invariant::StopGameStateInvariant>());

        keepPlayInvariants.clear();
        keepPlayInvariants.emplace_back(std::make_unique<invariant::StopGameStateInvariant>());

        roles = {
                std::make_unique<role::BallAvoider>("keeper"),
                std::make_unique<role::BallAvoider>("defender_0"),
                std::make_unique<role::BallAvoider>("defender_1"),
                std::make_unique<role::BallAvoider>("defender_2"),
                std::make_unique<role::BallAvoider>("mid_field_0"),
                std::make_unique<role::BallAvoider>("mid_field_1"),
                std::make_unique<role::BallAvoider>("mid_field_2"),
                std::make_unique<role::BallAvoider>("offender_0"),
                std::make_unique<role::BallAvoider>("offender_1"),
                std::make_unique<role::BallAvoider>("offender_2"),
                std::make_unique<role::BallAvoider>("offender_3")
        };
    }

uint8_t AggressiveFormation::score(world_new::World* world) noexcept { return 100; }

    void AggressiveFormation::calculateInfoForRoles() noexcept {
        // TODO: TUNE these positions could probably be a bit better once we decide how we want to play
        stpInfos["keeper"].setPositionToMoveTo(Vector2{-4.5, 0});
        stpInfos["defender_0"].setPositionToMoveTo(Vector2{-3, 3});
        stpInfos["defender_1"].setPositionToMoveTo(Vector2{-3, 0});
        stpInfos["defender_2"].setPositionToMoveTo(Vector2{-3, -3});
        stpInfos["mid_field_0"].setPositionToMoveTo(Vector2{-2, 3});
        stpInfos["mid_field_1"].setPositionToMoveTo(Vector2{-2, 0});
        stpInfos["mid_field_2"].setPositionToMoveTo(Vector2{-2, -3});
        stpInfos["offender_0"].setPositionToMoveTo(Vector2{-1, 4});
        stpInfos["offender_1"].setPositionToMoveTo(Vector2{-1, 1.5});
        stpInfos["offender_2"].setPositionToMoveTo(Vector2{-1, -1.5});
        stpInfos["offender_3"].setPositionToMoveTo(Vector2{-1, -4});
    }

    bool AggressiveFormation::shouldRoleSkipEndTactic() { return false; }

    Dealer::FlagMap AggressiveFormation::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
        Dealer::DealerFlag not_important(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

        flagMap.insert({"keeper", {keeperFlag}});
        flagMap.insert({"defender_0", {not_important}});
        flagMap.insert({"defender_1", {not_important}});
        flagMap.insert({"defender_2", {not_important}});
        flagMap.insert({"mid_field_0", {not_important}});
        flagMap.insert({"mid_field_1", {not_important}});
        flagMap.insert({"mid_field_2", {not_important}});
        flagMap.insert({"offender_0", {not_important}});
        flagMap.insert({"offender_1", {not_important}});
        flagMap.insert({"offender_2", {not_important}});
        flagMap.insert({"offender_3", {not_important}});

        return flagMap;
    }

    const char *AggressiveFormation::getName() { return "Aggressive Formation"; }
}  // namespace rtt::ai::stp::play