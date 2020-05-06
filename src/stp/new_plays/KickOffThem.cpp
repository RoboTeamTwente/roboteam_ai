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
        std::make_unique<role::Keeper>(role::Keeper("keeper")), std::make_unique<role::Halt>(role::Halt("halt_0")), std::make_unique<role::Halt>(role::Halt("halt_1")), std::make_unique<role::Halt>(role::Halt("halt_2")),
        std::make_unique<role::Halt>(role::Halt("halt_3")), std::make_unique<role::Halt>(role::Halt("halt_4")), std::make_unique<role::Halt>(role::Halt("halt_5")),
        std::make_unique<role::Halt>(role::Halt("halt_6")), std::make_unique<role::Halt>(role::Halt("halt_7")), std::make_unique<role::Halt>(role::Halt("halt_8")),
        std::make_unique<role::Halt>(role::Halt("halt_9"))};
}

uint8_t KickOffThem::score(world_new::World* world) noexcept { return 100; }

void KickOffThem::calculateInfoForRoles() noexcept {
    auto width = field.getFieldWidth();
    auto length = field.getFieldLength();

    // Keeper
    if (stpInfos.find("keeper") != stpInfos.end()) {
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));
        stpInfos["keeper"].setPositionToShootAt(Vector2{0.0, 0.0});
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    }

    // TODO: Improve positions
    // regular bots
    if (stpInfos.find("formation_0") != stpInfos.end()) {
        stpInfos["formation_0"].setPositionToMoveTo(Vector2(-length / 4 + 2, width / 8));
    }
    if (stpInfos.find("formation_1") != stpInfos.end()) {
        stpInfos["formation_1"].setPositionToMoveTo(Vector2(-length / 4 + 2, -width / 8));
    }
    if (stpInfos.find("formation_2") != stpInfos.end()) {
        stpInfos["formation_2"].setPositionToMoveTo(Vector2(-length / 8 + 2, width / 4));
    }
    if (stpInfos.find("formation_3") != stpInfos.end()) {
        stpInfos["formation_3"].setPositionToMoveTo(Vector2(-length / 8 + 2, -width / 4));
    }
    if (stpInfos.find("formation_4") != stpInfos.end()) {
        stpInfos["formation_4"].setPositionToMoveTo(Vector2(-length * 3 / 8 + 2, 0.0));
    }
    if (stpInfos.find("formation_5") != stpInfos.end()) {
        stpInfos["formation_5"].setPositionToMoveTo(Vector2(-length * 3 / 8 + 2, width / 5));
    }
    if (stpInfos.find("formation_6") != stpInfos.end()) {
        stpInfos["formation_6"].setPositionToMoveTo(Vector2(-length * 3 / 8 + 2, -width / 5));
    }
    if (stpInfos.find("formation_7") != stpInfos.end()) {
        stpInfos["formation_7"].setPositionToMoveTo(Vector2(-length / 4 + 2, width / 3));
    }
    if (stpInfos.find("formation_8") != stpInfos.end()) {
        stpInfos["formation_8"].setPositionToMoveTo(Vector2(-length / 4 + 2, -width / 3));
    }
    if (stpInfos.find("formation_9") != stpInfos.end()) {
        stpInfos["formation_9"].setPositionToMoveTo(Vector2(-length / 4 + 2, -width / 3));
    }
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