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