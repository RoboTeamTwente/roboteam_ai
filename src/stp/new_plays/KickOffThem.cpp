//
// Created by timovdk on 5/1/20.
//

#include "stp/new_plays/KickOffThem.h"

#include "stp/invariants/game_states/KickOffThemGameStateInvariant.h"
#include "stp/new_roles/Halt.h"
#include "stp/new_roles/Keeper.h"

namespace rtt::ai::stp::play {

KickOffThem::KickOffThem() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::KickOffThemGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::KickOffThemGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")), std::make_unique<role::Halt>(role::Halt("halt_0")), std::make_unique<role::Halt>(role::Halt("halt_1")),
        std::make_unique<role::Halt>(role::Halt("halt_2")),     std::make_unique<role::Halt>(role::Halt("halt_3")), std::make_unique<role::Halt>(role::Halt("halt_4")),
        std::make_unique<role::Halt>(role::Halt("halt_5")),     std::make_unique<role::Halt>(role::Halt("halt_6")), std::make_unique<role::Halt>(role::Halt("halt_7")),
        std::make_unique<role::Halt>(role::Halt("halt_8")),     std::make_unique<role::Halt>(role::Halt("halt_9"))};
}

uint8_t KickOffThem::score(world_new::World* world) noexcept { return 100; }

void KickOffThem::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));
    stpInfos["keeper"].setPositionToShootAt(Vector2{0.0, 0.0});
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
}

bool KickOffThem::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap KickOffThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {keeperFlag}});
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
    return flagMap;
}

const char* KickOffThem::getName() { return "Kick Off Them"; }

}  // namespace rtt::ai::stp::play
