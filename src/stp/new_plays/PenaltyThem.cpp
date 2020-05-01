//
// Created by timovdk on 4/28/20.
//

#include <stp/invariants/BallCloseToUsInvariant.h>
#include <stp/invariants/BallMovesSlowInvariant.h>
#include <stp/new_plays/PenaltyThem.h>
#include <stp/new_roles/Halt.h>
#include <stp/new_roles/PenaltyKeeper.h>

namespace rtt::ai::stp::play {

PenaltyThem::PenaltyThem() : Play() {
    // TODO: decide start invariants
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToUsInvariant>());

    // TODO: decide keep invariants
    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallMovesSlowInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::PenaltyKeeper>(role::PenaltyKeeper("keeper")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_0")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_1")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_2")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_3")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_4")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_5")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_6")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_7")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_8")),
                                                                                       std::make_unique<role::Halt>(role::Halt("halt_9"))};
}

uint8_t PenaltyThem::score(world_new::World *world) noexcept { return 111; }

Dealer::FlagMap PenaltyThem::decideRoleFlags() const noexcept {
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

void PenaltyThem::calculateInfoForRoles() noexcept {
    // TODO: these positions might need to change in the future
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setPositionToShootAt(Vector2{-2, -4});
}

bool PenaltyThem::shouldRoleSkipEndTactic() { return false; }

const char *PenaltyThem::getName() { return "Penalty Them Play"; }

}  // namespace rtt::ai::stp::play