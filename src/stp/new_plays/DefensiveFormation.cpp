//
// Created by timo on 3/27/20.
//

#include "stp/new_plays/DefensiveFormation.h"

#include "stp/invariants/game_states/StopGameStateInvariant.h"
#include "stp/new_roles/Formation.h"

namespace rtt::ai::stp::play {

DefensiveFormation::DefensiveFormation() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::StopGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::StopGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Formation>(role::Formation("keeper")),      std::make_unique<role::Formation>(role::Formation("defender_0")),
        std::make_unique<role::Formation>(role::Formation("defender_1")),  std::make_unique<role::Formation>(role::Formation("defender_2")),
        std::make_unique<role::Formation>(role::Formation("defender_3")),  std::make_unique<role::Formation>(role::Formation("mid_field_0")),
        std::make_unique<role::Formation>(role::Formation("mid_field_1")), std::make_unique<role::Formation>(role::Formation("mid_field_2")),
        std::make_unique<role::Formation>(role::Formation("offender_0")),  std::make_unique<role::Formation>(role::Formation("offender_1")),
        std::make_unique<role::Formation>(role::Formation("offender_2"))};
}

uint8_t DefensiveFormation::score(world_new::World* world) noexcept { return 20; }

void DefensiveFormation::calculateInfoForRoles() noexcept {
    // TODO: TUNE these positions could probably be a bit better once we decide how we want to play
    stpInfos["keeper"].setPositionToMoveTo(Vector2{-4.5, 0});
    stpInfos["defender_0"].setPositionToMoveTo(Vector2{-3, 4});
    stpInfos["defender_1"].setPositionToMoveTo(Vector2{-3, 1});
    stpInfos["defender_2"].setPositionToMoveTo(Vector2{-3, -1});
    stpInfos["defender_3"].setPositionToMoveTo(Vector2{-3, -4});
    stpInfos["mid_field_0"].setPositionToMoveTo(Vector2{-2, 3});
    stpInfos["mid_field_1"].setPositionToMoveTo(Vector2{-2, 0});
    stpInfos["mid_field_2"].setPositionToMoveTo(Vector2{-2, -3});
    stpInfos["offender_0"].setPositionToMoveTo(Vector2{-1, 4});
    stpInfos["offender_1"].setPositionToMoveTo(Vector2{-1, 0});
    stpInfos["offender_2"].setPositionToMoveTo(Vector2{-1, -4});
}

bool DefensiveFormation::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap DefensiveFormation::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"defender_0", {not_important}});
    flagMap.insert({"defender_1", {not_important}});
    flagMap.insert({"defender_2", {not_important}});
    flagMap.insert({"defender_3", {not_important}});
    flagMap.insert({"mid_field_0", {not_important}});
    flagMap.insert({"mid_field_1", {not_important}});
    flagMap.insert({"mid_field_2", {not_important}});
    flagMap.insert({"offender_0", {not_important}});
    flagMap.insert({"offender_1", {not_important}});
    flagMap.insert({"offender_2", {not_important}});

    return flagMap;
}
const char* DefensiveFormation::getName() { return "Defensive Formation"; }
}  // namespace rtt::ai::stp::play