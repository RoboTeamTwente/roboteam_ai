//
// Created by timovdk on 5/1/20.
//

#include "stp/new_plays/PenaltyThemPrepare.h"

#include "stp/invariants/game_states/PenaltyThemPrepareGameStateInvariant.h"
#include "stp/new_roles/Formation.h"

namespace rtt::ai::stp::play {

PenaltyThemPrepare::PenaltyThemPrepare() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyThemPrepareGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyThemPrepareGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Formation>(role::Formation("keeper")),
                                                                                 std::make_unique<role::Formation>(role::Formation("formation_0")),
                                                                                 std::make_unique<role::Formation>(role::Formation("formation_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("formation_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("formation_3")),
                                                                                 std::make_unique<role::Formation>(role::Formation("formation_4")),
                                                                                 std::make_unique<role::Formation>(role::Formation("formation_5")),
                                                                                 std::make_unique<role::Formation>(role::Formation("formation_6")),
                                                                                 std::make_unique<role::Formation>(role::Formation("formation_7")),
                                                                                 std::make_unique<role::Formation>(role::Formation("formation_8")),
                                                                                 std::make_unique<role::Formation>(role::Formation("formation_9"))};
}

uint8_t PenaltyThemPrepare::score(world_new::World* world) noexcept { return 100; }

void PenaltyThemPrepare::calculateInfoForRoles() noexcept {
    auto width = field.getFieldWidth();
    auto length = field.getFieldLength();

    // Keeper
    if (stpInfos.find("keeper") != stpInfos.end()) {
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));
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
        stpInfos["formation_9"].setPositionToMoveTo(Vector2(-length/8 + 2, 0.0));
    }
}

bool PenaltyThemPrepare::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap PenaltyThemPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"formation_0", {}});
    flagMap.insert({"formation_1", {}});
    flagMap.insert({"formation_2", {}});
    flagMap.insert({"formation_3", {}});
    flagMap.insert({"formation_4", {}});
    flagMap.insert({"formation_5", {}});
    flagMap.insert({"formation_6", {}});
    flagMap.insert({"formation_7", {}});
    flagMap.insert({"formation_8", {}});
    flagMap.insert({"formation_9", {}});
    return flagMap;
}

const char* PenaltyThemPrepare::getName() { return "Penalty Them Prepare Play"; }

}  // namespace rtt::ai::stp::play
