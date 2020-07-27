//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/PenaltyUs.h"

#include "stp/invariants/game_states/PenaltyUsGameStateInvariant.h"
#include "stp/roles/Attacker.h"
#include "stp/roles/Halt.h"
#include "stp/roles/PenaltyKeeper.h"

namespace rtt::ai::stp::play {

PenaltyUs::PenaltyUs() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyUsGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyUsGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::PenaltyKeeper>(role::PenaltyKeeper("keeper")),
                                                                                       std::make_unique<role::Attacker>(role::Attacker("kicker")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_0")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_1")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_2")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_3")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_4")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_5")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_6")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_7")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_8"))};
}

uint8_t PenaltyUs::score(world_new::World *world) noexcept { return 100; }

Dealer::FlagMap PenaltyUs::decideRoleFlags() const noexcept {
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

void PenaltyUs::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());

    // TODO: the shoot position might need to change
    stpInfos["keeper"].setPositionToShootAt(Vector2{0, -2});

    // TODO: the shoot position might need to change
    stpInfos["kicker"].setPositionToShootAt(field.getTheirGoalCenter() + Vector2{1.0, 0.5});
    stpInfos["kicker"].setShotType(ShotType::MAX);
}

bool PenaltyUs::shouldRoleSkipEndTactic() { return false; }

const char *PenaltyUs::getName() { return "Penalty Us"; }

}  // namespace rtt::ai::stp::play
