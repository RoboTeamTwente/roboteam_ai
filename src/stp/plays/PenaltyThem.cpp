//
// Created by timovdk on 4/28/20.
//

#include "stp/plays/PenaltyThem.h"

#include "stp/invariants/game_states/PenaltyThemGameStateInvariant.h"
#include "stp/roles/Halt.h"
#include "stp/roles/PenaltyKeeper.h"

namespace rtt::ai::stp::play {

PenaltyThem::PenaltyThem() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyThemGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyThemGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::PenaltyKeeper>("keeper"),
        std::make_unique<role::Halt>("halt_0"),
        std::make_unique<role::Halt>("halt_1"),
        std::make_unique<role::Halt>("halt_2"),
        std::make_unique<role::Halt>("halt_3"),
        std::make_unique<role::Halt>("halt_4"),
        std::make_unique<role::Halt>("halt_5"),
        std::make_unique<role::Halt>("halt_6"),
        std::make_unique<role::Halt>("halt_7"),
        std::make_unique<role::Halt>("halt_8"),
       std::make_unique<role::Halt>("halt_9")};
}

uint8_t PenaltyThem::score(world::World *world) noexcept { return 100; }

Dealer::FlagMap PenaltyThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"halt_0", {notImportant}});
    flagMap.insert({"halt_1", {notImportant}});
    flagMap.insert({"halt_2", {notImportant}});
    flagMap.insert({"halt_3", {notImportant}});
    flagMap.insert({"halt_4", {notImportant}});
    flagMap.insert({"halt_5", {notImportant}});
    flagMap.insert({"halt_6", {notImportant}});
    flagMap.insert({"halt_7", {notImportant}});
    flagMap.insert({"halt_8", {notImportant}});
    flagMap.insert({"halt_9", {notImportant}});

    return flagMap;
}

void PenaltyThem::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());

    // TODO: the shoot position might need to change
    stpInfos["keeper"].setPositionToShootAt(Vector2{0, -2});
}

bool PenaltyThem::shouldRoleSkipEndTactic() { return false; }

const char *PenaltyThem::getName() { return "Penalty Them"; }

}  // namespace rtt::ai::stp::play