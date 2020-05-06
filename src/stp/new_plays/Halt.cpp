//
// Created by jessevw on 24.03.20.
//

#include "stp/new_plays/Halt.h"

#include "stp/invariants/game_states/HaltGameStateInvariant.h"
#include "stp/new_roles/Halt.h"

namespace rtt::ai::stp::play {

Halt::Halt() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::HaltGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::HaltGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Halt>(role::Halt("halt_0")), std::make_unique<role::Halt>(role::Halt("halt_1")), std::make_unique<role::Halt>(role::Halt("halt_2")),
        std::make_unique<role::Halt>(role::Halt("halt_3")), std::make_unique<role::Halt>(role::Halt("halt_4")), std::make_unique<role::Halt>(role::Halt("halt_5")),
        std::make_unique<role::Halt>(role::Halt("halt_6")), std::make_unique<role::Halt>(role::Halt("halt_7")), std::make_unique<role::Halt>(role::Halt("halt_8")),
        std::make_unique<role::Halt>(role::Halt("halt_9")), std::make_unique<role::Halt>(role::Halt("halt_10"))};
}

uint8_t Halt::score(world_new::World* world) noexcept { return 100; }

void Halt::calculateInfoForRoles() noexcept {}

bool Halt::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap Halt::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"halt_0", {}});
    flagMap.insert({"halt_1", {}});
    flagMap.insert({"halt_2", {}});
    flagMap.insert({"halt_3", {}});
    flagMap.insert({"halt_4", {}});
    flagMap.insert({"halt_5", {}});
    flagMap.insert({"halt_6", {}});
    flagMap.insert({"halt_7", {}});
    flagMap.insert({"halt_8", {}});
    flagMap.insert({"halt_9", {}});
    flagMap.insert({"halt_10", {}});
    return flagMap;
}

const char* Halt::getName() { return "Halt"; }

}  // namespace rtt::ai::stp::play
