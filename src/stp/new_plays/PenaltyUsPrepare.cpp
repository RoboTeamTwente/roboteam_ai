//
// Created by timovdk on 5/1/20.
//
#include "stp/new_plays/PenaltyUsPrepare.h"

#include "stp/invariants/game_states/PenaltyUsPrepareGameStateInvariant.h"
#include "stp/new_roles/Formation.h"

namespace rtt::ai::stp::play {

PenaltyUsPrepare::PenaltyUsPrepare() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyUsPrepareGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyUsPrepareGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Formation>(role::Formation("keeper")),      std::make_unique<role::Formation>(role::Formation("kicker_formation")),
        std::make_unique<role::Formation>(role::Formation("formation_0")), std::make_unique<role::Formation>(role::Formation("formation_1")),
        std::make_unique<role::Formation>(role::Formation("formation_2")), std::make_unique<role::Formation>(role::Formation("formation_3")),
        std::make_unique<role::Formation>(role::Formation("formation_4")), std::make_unique<role::Formation>(role::Formation("formation_5")),
        std::make_unique<role::Formation>(role::Formation("formation_6")), std::make_unique<role::Formation>(role::Formation("formation_7")),
        std::make_unique<role::Formation>(role::Formation("formation_8"))};
}

uint8_t PenaltyUsPrepare::score(world_new::World* world) noexcept { return 100; }

void PenaltyUsPrepare::calculateInfoForRoles() noexcept {
    auto width = field.getFieldWidth();
    auto length = field.getFieldLength();

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));

    // kicker, position right behind the ball
    stpInfos["kicker_formation"].setPositionToMoveTo(world->getWorld()->getBall()->get()->getPos() - Vector2{0.25, 0.0});

    // TODO: Improve positions
    // regular bots
    stpInfos["formation_0"].setPositionToMoveTo(Vector2(-length / 4 + 2, width / 8));
    stpInfos["formation_1"].setPositionToMoveTo(Vector2(-length / 4 + 2, -width / 8));
    stpInfos["formation_2"].setPositionToMoveTo(Vector2(-length / 8 + 2, width / 4));
    stpInfos["formation_3"].setPositionToMoveTo(Vector2(-length / 8 + 2, -width / 4));
    stpInfos["formation_4"].setPositionToMoveTo(Vector2(-length * 3 / 8 + 2, 0.0));
    stpInfos["formation_5"].setPositionToMoveTo(Vector2(-length * 3 / 8 + 2, width / 5));
    stpInfos["formation_6"].setPositionToMoveTo(Vector2(-length * 3 / 8 + 2, -width / 5));
    stpInfos["formation_7"].setPositionToMoveTo(Vector2(-length / 4 + 2, width / 3));
    stpInfos["formation_8"].setPositionToMoveTo(Vector2(-length / 4 + 2, -width / 3));
}

bool PenaltyUsPrepare::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap PenaltyUsPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag kickerFormationFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag not_important(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"kicker_formation", {kickerFormationFlag}});
    flagMap.insert({"formation_0", {not_important}});
    flagMap.insert({"formation_1", {not_important}});
    flagMap.insert({"formation_2", {not_important}});
    flagMap.insert({"formation_3", {not_important}});
    flagMap.insert({"formation_4", {not_important}});
    flagMap.insert({"formation_5", {not_important}});
    flagMap.insert({"formation_6", {not_important}});
    flagMap.insert({"formation_7", {not_important}});
    flagMap.insert({"formation_8", {not_important}});
    return flagMap;
}

const char* PenaltyUsPrepare::getName() { return "Penalty Us Prepare"; }

}  // namespace rtt::ai::stp::play
