//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/PenaltyUsPrepare.h"

#include "stp/invariants/game_states/PenaltyUsPrepareGameStateInvariant.h"
#include "stp/roles/Formation.h"

namespace rtt::ai::stp::play {

PenaltyUsPrepare::PenaltyUsPrepare() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyUsPrepareGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::PenaltyUsPrepareGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Formation>("keeper"),
        std::make_unique<role::Formation>("kicker_formation"),
        std::make_unique<role::Formation>("formation_0"),
        std::make_unique<role::Formation>("formation_1"),
        std::make_unique<role::Formation>("formation_2"),
        std::make_unique<role::Formation>("formation_3"),
        std::make_unique<role::Formation>("formation_4"),
        std::make_unique<role::Formation>("formation_5"),
        std::make_unique<role::Formation>("formation_6"),
        std::make_unique<role::Formation>("formation_7"),
        std::make_unique<role::Formation>("formation_8")};
}

uint8_t PenaltyUsPrepare::score(world::World* world) noexcept { return 100; }

void PenaltyUsPrepare::calculateInfoForRoles() noexcept {
    const double xPosition = -4 * control_constants::ROBOT_RADIUS;
    const double distanceToCenterLine = field.getFieldWidth() / 2 - 2*control_constants::ROBOT_RADIUS;
    const double yPosition = Constants::STD_TIMEOUT_TO_TOP() ? distanceToCenterLine: -distanceToCenterLine;

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter()));

    // kicker, position right behind the ball
    stpInfos["kicker_formation"].setPositionToMoveTo(world->getWorld()->getBall()->get()->getPos() - Vector2{0.25, 0.0});

    // regular bots
    const std::string formation = "formation_";
    for(int i = 1; i <= 9; i++) {
        stpInfos[formation + std::to_string(i - 1)].setPositionToMoveTo(Vector2(i * xPosition, yPosition));
        stpInfos[formation + std::to_string(i - 1)].setBallAvoidanceDistance(0.6);
    }
}

bool PenaltyUsPrepare::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap PenaltyUsPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag kickerFormationFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"kicker_formation", {kickerFormationFlag}});
    flagMap.insert({"formation_0", {notImportant}});
    flagMap.insert({"formation_1", {notImportant}});
    flagMap.insert({"formation_2", {notImportant}});
    flagMap.insert({"formation_3", {notImportant}});
    flagMap.insert({"formation_4", {notImportant}});
    flagMap.insert({"formation_5", {notImportant}});
    flagMap.insert({"formation_6", {notImportant}});
    flagMap.insert({"formation_7", {notImportant}});
    flagMap.insert({"formation_8", {notImportant}});
    return flagMap;
}

const char* PenaltyUsPrepare::getName() { return "Penalty Us Prepare"; }

}  // namespace rtt::ai::stp::play
