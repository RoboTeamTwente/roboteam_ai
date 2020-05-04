//
// Created by timovdk on 5/1/20.
//

#include "stp/new_plays/KickOffUs.h"

#include "stp/invariants/game_states/KickOffUsGameStateInvariant.h"
#include "stp/new_roles/Attacker.h"
#include "stp/new_roles/Halt.h"
#include "stp/new_roles/Keeper.h"

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

uint8_t KickOffUs::score(world_new::World* world) noexcept { return 100; }

void KickOffUs::calculateInfoForRoles() noexcept {
    auto width = field.getFieldWidth();
    auto length = field.getFieldLength();

    // Keeper
    if (stpInfos.find("keeper") != stpInfos.end()) {
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));
        stpInfos["keeper"].setPositionToShootAt(Vector2{0.0, 0.0});
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    }

    // Kicker
    if (stpInfos.find("kicker") != stpInfos.end()) {
        stpInfos["kicker"].setPositionToShootAt(Vector2{3.0, 2.0});
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
}

bool KickOffUs::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap KickOffUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"kicker", {kickerFlag}});
    flagMap.insert({"halt_0", {}});
    flagMap.insert({"halt_1", {}});
    flagMap.insert({"halt_2", {}});
    flagMap.insert({"halt_3", {}});
    flagMap.insert({"halt_4", {}});
    flagMap.insert({"halt_5", {}});
    flagMap.insert({"halt_6", {}});
    flagMap.insert({"halt_7", {}});
    flagMap.insert({"halt_8", {}});
    return flagMap;
}

const char* KickOffUs::getName() { return "Kick Off Us"; }

}  // namespace rtt::ai::stp::play
