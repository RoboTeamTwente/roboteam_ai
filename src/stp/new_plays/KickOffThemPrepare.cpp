//
// Created by jordi on 30-04-20.
//

#include "stp/new_plays/KickOffThemPrepare.h"

#include "stp/invariants/game_states/KickOffThemPrepareGameStateInvariant.h"
#include "stp/new_roles/Formation.h"

namespace rtt::ai::stp::play {

KickOffThemPrepare::KickOffThemPrepare() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::KickOffThemPrepareGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::KickOffThemPrepareGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Formation>(role::Formation("keeper")),      std::make_unique<role::Formation>(role::Formation("formation_0")),
        std::make_unique<role::Formation>(role::Formation("formation_1")), std::make_unique<role::Formation>(role::Formation("formation_2")),
        std::make_unique<role::Formation>(role::Formation("formation_3")), std::make_unique<role::Formation>(role::Formation("formation_4")),
        std::make_unique<role::Formation>(role::Formation("formation_5")), std::make_unique<role::Formation>(role::Formation("formation_6")),
        std::make_unique<role::Formation>(role::Formation("formation_7")), std::make_unique<role::Formation>(role::Formation("formation_8")),
        std::make_unique<role::Formation>(role::Formation("formation_9"))};
}

uint8_t KickOffThemPrepare::score(world_new::World* world) noexcept { return 100; }

void KickOffThemPrepare::calculateInfoForRoles() noexcept {
    auto width = field.getFieldWidth();
    auto length = field.getFieldLength();

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));

    // regular bots
    stpInfos["formation_0"].setPositionToMoveTo(Vector2(-length / 4, width / 8));
    stpInfos["formation_1"].setPositionToMoveTo(Vector2(-length / 4, -width / 8));
    stpInfos["formation_2"].setPositionToMoveTo(Vector2(-length / 8, width / 4));
    stpInfos["formation_3"].setPositionToMoveTo(Vector2(-length / 8, -width / 4));
    stpInfos["formation_4"].setPositionToMoveTo(Vector2(-length * 3 / 8, 0.0));
    stpInfos["formation_5"].setPositionToMoveTo(Vector2(-length * 3 / 8, width / 5));
    stpInfos["formation_6"].setPositionToMoveTo(Vector2(-length * 3 / 8, -width / 5));
    stpInfos["formation_7"].setPositionToMoveTo(Vector2(-length / 4, width / 3));
    stpInfos["formation_8"].setPositionToMoveTo(Vector2(-length / 4, -width / 3));
    stpInfos["formation_9"].setPositionToMoveTo(Vector2(-length / 4, -width / 3));
}

bool KickOffThemPrepare::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap KickOffThemPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag not_important(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"formation_0", {not_important}});
    flagMap.insert({"formation_1", {not_important}});
    flagMap.insert({"formation_2", {not_important}});
    flagMap.insert({"formation_3", {not_important}});
    flagMap.insert({"formation_4", {not_important}});
    flagMap.insert({"formation_5", {not_important}});
    flagMap.insert({"formation_6", {not_important}});
    flagMap.insert({"formation_7", {not_important}});
    flagMap.insert({"formation_8", {not_important}});
    flagMap.insert({"formation_9", {not_important}});

    return flagMap;
}

const char* KickOffThemPrepare::getName() { return "Kick Off Them Prepare"; }

}  // namespace rtt::ai::stp::play