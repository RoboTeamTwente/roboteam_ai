//
// Created by jordi on 30-04-20.
//

#include "include/roboteam_ai/stp/plays/referee_specific/KickOffUsPrepare.h"

#include "stp/invariants/game_states/KickOffUsPrepareGameStateEvaluation.h"
#include "include/roboteam_ai/stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

KickOffUsPrepare::KickOffUsPrepare() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::KickOffUsPrepareGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::KickOffUsPrepareGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Formation>(role::Formation("keeper")),      std::make_unique<role::Formation>(role::Formation("formation_0")),
        std::make_unique<role::Formation>(role::Formation("formation_1")), std::make_unique<role::Formation>(role::Formation("formation_2")),
        std::make_unique<role::Formation>(role::Formation("formation_3")), std::make_unique<role::Formation>(role::Formation("formation_4")),
        std::make_unique<role::Formation>(role::Formation("formation_5")), std::make_unique<role::Formation>(role::Formation("formation_6")),
        std::make_unique<role::Formation>(role::Formation("formation_7")), std::make_unique<role::Formation>(role::Formation("formation_8")),
        std::make_unique<role::Formation>(role::Formation("formation_9"))};
}

uint8_t KickOffUsPrepare::score(world::World* world) noexcept { return 100; }

void KickOffUsPrepare::calculateInfoForRoles() noexcept {
    auto width = field.getFieldWidth();
    auto length = field.getFieldLength();

    // Keeper
    if (stpInfos.find("keeper") != stpInfos.end()) {
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));
    }

    // Positions of the kick off us formation which will be dealt to the Formation roles in order
    // Formation_0 will go to the ball
    stpInfos["formation_0"].setPositionToMoveTo(Vector2(0.25, 0.0));
    stpInfos["formation_1"].setPositionToMoveTo(Vector2(-length / 4, width / 8));
    stpInfos["formation_2"].setPositionToMoveTo(Vector2(-length / 4, -width / 8));
    stpInfos["formation_3"].setPositionToMoveTo(Vector2(-length / 8, width / 4));
    stpInfos["formation_4"].setPositionToMoveTo(Vector2(-length / 8, -width / 4));
    stpInfos["formation_5"].setPositionToMoveTo(Vector2(-length * 3 / 8, 0.0));
    stpInfos["formation_6"].setPositionToMoveTo(Vector2(-length * 3 / 8, width / 5));
    stpInfos["formation_7"].setPositionToMoveTo(Vector2(-length * 3 / 8, -width / 5));
    stpInfos["formation_8"].setPositionToMoveTo(Vector2(-length / 4, width / 3));
    stpInfos["formation_9"].setPositionToMoveTo(Vector2(-length / 4, -width / 3));
}

bool KickOffUsPrepare::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap KickOffUsPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::UNIQUE);
    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"formation_0", {kickerFlag}});
    flagMap.insert({"formation_1", {notImportant}});
    flagMap.insert({"formation_2", {notImportant}});
    flagMap.insert({"formation_3", {notImportant}});
    flagMap.insert({"formation_4", {notImportant}});
    flagMap.insert({"formation_5", {notImportant}});
    flagMap.insert({"formation_6", {notImportant}});
    flagMap.insert({"formation_7", {notImportant}});
    flagMap.insert({"formation_8", {notImportant}});
    flagMap.insert({"formation_9", {notImportant}});

    return flagMap;
}

const char* KickOffUsPrepare::getName() { return "Kick Off Us Prepare"; }

}  // namespace rtt::ai::stp::play