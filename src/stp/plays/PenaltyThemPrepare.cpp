//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/PenaltyThemPrepare.h"

#include "stp/invariants/game_states/PenaltyThemPrepareGameStateInvariant.h"
#include "stp/roles/Formation.h"

namespace rtt::ai::stp::play {

PenaltyThemPrepare::PenaltyThemPrepare() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyThemPrepareGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyThemPrepareGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Formation>(role::Formation("keeper")),      std::make_unique<role::Formation>(role::Formation("formation_0")),
        std::make_unique<role::Formation>(role::Formation("formation_1")), std::make_unique<role::Formation>(role::Formation("formation_2")),
        std::make_unique<role::Formation>(role::Formation("formation_3")), std::make_unique<role::Formation>(role::Formation("formation_4")),
        std::make_unique<role::Formation>(role::Formation("formation_5")), std::make_unique<role::Formation>(role::Formation("formation_6")),
        std::make_unique<role::Formation>(role::Formation("formation_7")), std::make_unique<role::Formation>(role::Formation("formation_8")),
        std::make_unique<role::Formation>(role::Formation("formation_9"))};
}

uint8_t PenaltyThemPrepare::score(world::World* world) noexcept { return 100; }

void PenaltyThemPrepare::calculateInfoForRoles() noexcept {
    const auto xPosition = -4 * control_constants::ROBOT_RADIUS;
    const auto yPosition = Constants::STD_TIMEOUT_TO_TOP() ? field.getFieldWidth() / 2.2 : -field.getFieldWidth() / 2.2;

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));

    // regular bots
    stpInfos["formation_0"].setPositionToMoveTo(Vector2(xPosition, yPosition));
    stpInfos["formation_1"].setPositionToMoveTo(Vector2(xPosition - 4 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["formation_2"].setPositionToMoveTo(Vector2(xPosition - 8 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["formation_3"].setPositionToMoveTo(Vector2(xPosition - 12 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["formation_4"].setPositionToMoveTo(Vector2(xPosition - 16 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["formation_5"].setPositionToMoveTo(Vector2(xPosition - 20 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["formation_6"].setPositionToMoveTo(Vector2(xPosition - 24 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["formation_7"].setPositionToMoveTo(Vector2(xPosition - 28 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["formation_8"].setPositionToMoveTo(Vector2(xPosition - 32 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["formation_9"].setPositionToMoveTo(Vector2(xPosition - 36 * control_constants::ROBOT_RADIUS, yPosition));
}

bool PenaltyThemPrepare::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap PenaltyThemPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"formation_0", {notImportant}});
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

const char* PenaltyThemPrepare::getName() { return "Penalty Them Prepare"; }

}  // namespace rtt::ai::stp::play
