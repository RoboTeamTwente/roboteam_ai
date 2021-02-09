//
// Created by timovdk on 5/1/20.
//

#include "include/roboteam_ai/stp/plays/referee_specific/PenaltyThemPrepare.h"

#include "stp/invariants/game_states/PenaltyThemPrepareGameStateInvariant.h"
#include "include/roboteam_ai/stp/roles/passive/Formation.h"

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
    const double xPosition = -4 * control_constants::ROBOT_RADIUS;
    const double distanceToCenterLine = field.getFieldWidth() / 2 - 2*control_constants::ROBOT_RADIUS;
    const double yPosition = Constants::STD_TIMEOUT_TO_TOP() ? distanceToCenterLine: -distanceToCenterLine;

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));

    // regular bots
    const std::string formation = "formation_";
    for(int i = 1; i <= 10; i++) {
        stpInfos[formation + std::to_string(i - 1)].setPositionToMoveTo(Vector2(i * xPosition, yPosition));
    }
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
