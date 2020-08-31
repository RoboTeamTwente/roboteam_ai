//
// Created by timovdk on 3/30/20.
//

#include "stp/plays/AggressiveFormation.h"

#include "stp/invariants/game_states/StopGameStateInvariant.h"
#include "stp/roles/BallAvoider.h"

namespace rtt::ai::stp::play {

AggressiveFormation::AggressiveFormation() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::StopGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::StopGameStateInvariant>());

    roles = {std::make_unique<role::BallAvoider>("keeper"),      std::make_unique<role::BallAvoider>("defender_0"),  std::make_unique<role::BallAvoider>("defender_1"),
             std::make_unique<role::BallAvoider>("defender_2"),  std::make_unique<role::BallAvoider>("mid_field_0"), std::make_unique<role::BallAvoider>("mid_field_1"),
             std::make_unique<role::BallAvoider>("mid_field_2"), std::make_unique<role::BallAvoider>("offender_0"),  std::make_unique<role::BallAvoider>("offender_1"),
             std::make_unique<role::BallAvoider>("offender_2"),  std::make_unique<role::BallAvoider>("offender_3")};
}

uint8_t AggressiveFormation::score(world::World* world) noexcept { return 100; }

void AggressiveFormation::calculateInfoForRoles() noexcept {
    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter() + Vector2{0.5, 0.0});
    stpInfos["defender_0"].setPositionToMoveTo(Vector2{-length / 5, 0.0});
    stpInfos["defender_1"].setPositionToMoveTo(Vector2{-length / 5, width / 6});
    stpInfos["defender_2"].setPositionToMoveTo(Vector2{-length / 5, -width / 6});
    stpInfos["mid_field_0"].setPositionToMoveTo(Vector2{-length / 8, 0.0});
    stpInfos["mid_field_1"].setPositionToMoveTo(Vector2{-length / 9, -width / 4});
    stpInfos["mid_field_2"].setPositionToMoveTo(Vector2{-length / 9, width / 4});
    stpInfos["offender_0"].setPositionToMoveTo(Vector2{length / 4, 0.0});
    stpInfos["offender_1"].setPositionToMoveTo(Vector2{length / 4, width / 4});
    stpInfos["offender_2"].setPositionToMoveTo(Vector2{length / 4, -width / 4});
    stpInfos["offender_3"].setPositionToMoveTo(Vector2{length / 3, 0.0});
}

bool AggressiveFormation::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap AggressiveFormation::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"defender_0", {notImportant}});
    flagMap.insert({"defender_1", {notImportant}});
    flagMap.insert({"defender_2", {notImportant}});
    flagMap.insert({"mid_field_0", {notImportant}});
    flagMap.insert({"mid_field_1", {notImportant}});
    flagMap.insert({"mid_field_2", {notImportant}});
    flagMap.insert({"offender_0", {notImportant}});
    flagMap.insert({"offender_1", {notImportant}});
    flagMap.insert({"offender_2", {notImportant}});
    flagMap.insert({"offender_3", {notImportant}});

    return flagMap;
}

const char* AggressiveFormation::getName() { return "Aggressive Formation"; }
}  // namespace rtt::ai::stp::play