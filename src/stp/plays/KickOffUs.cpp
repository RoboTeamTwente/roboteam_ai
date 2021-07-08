//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/KickOffUs.h"

#include "stp/invariants/game_states/KickOffUsGameStateInvariant.h"
#include "stp/roles/Attacker.h"
#include "stp/roles/Halt.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

KickOffUs::KickOffUs() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::KickOffUsGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::KickOffUsGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")), std::make_unique<role::Attacker>(role::Attacker("kicker")), std::make_unique<role::Halt>(role::Halt("halt_0")),
        std::make_unique<role::Halt>(role::Halt("halt_1")),     std::make_unique<role::Halt>(role::Halt("halt_2")),         std::make_unique<role::Halt>(role::Halt("halt_3")),
        std::make_unique<role::Halt>(role::Halt("halt_4")),     std::make_unique<role::Halt>(role::Halt("halt_5")),         std::make_unique<role::Halt>(role::Halt("halt_6")),
        std::make_unique<role::Halt>(role::Halt("halt_7")),     std::make_unique<role::Halt>(role::Halt("halt_8"))};
}

uint8_t KickOffUs::score(world::World* world) noexcept { return 100; }

void KickOffUs::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));
    stpInfos["keeper"].setPositionToShootAt(Vector2{0.0, 0.0});
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    // Kicker
    stpInfos["kicker"].setPositionToShootAt(Vector2{-1.0, 0.0});
    stpInfos["kicker"].setShotType(ShotType::PASS);
    stpInfos["kicker"].setBallAvoidanceDistance(0.0);
}

bool KickOffUs::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap KickOffUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"kicker", {kickerFlag}});
    flagMap.insert({"halt_0", {notImportant}});
    flagMap.insert({"halt_1", {notImportant}});
    flagMap.insert({"halt_2", {notImportant}});
    flagMap.insert({"halt_3", {notImportant}});
    flagMap.insert({"halt_4", {notImportant}});
    flagMap.insert({"halt_5", {notImportant}});
    flagMap.insert({"halt_6", {notImportant}});
    flagMap.insert({"halt_7", {notImportant}});
    flagMap.insert({"halt_8", {notImportant}});

    return flagMap;
}

const char* KickOffUs::getName() { return "Kick Off Us"; }

}  // namespace rtt::ai::stp::play
