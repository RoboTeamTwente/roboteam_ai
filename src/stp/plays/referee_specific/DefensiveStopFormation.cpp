//
// Created by timo on 3/27/20.
//

#include "include/roboteam_ai/stp/plays/referee_specific/DefensiveStopFormation.h"

#include "stp/invariants/game_states/StopGameStateEvaluation.h"
#include "stp/invariants/BallOnOurSideGlobalEvaluation.h"
#include "include/roboteam_ai/stp/roles/passive/BallAvoider.h"

namespace rtt::ai::stp::play {

DefensiveStopFormation::DefensiveStopFormation() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::StopGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallOnOurSideInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::StopGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::BallAvoider>(role::BallAvoider("keeper")),      std::make_unique<role::BallAvoider>(role::BallAvoider("defender_0")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("defender_1")),  std::make_unique<role::BallAvoider>(role::BallAvoider("defender_2")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("defender_3")),  std::make_unique<role::BallAvoider>(role::BallAvoider("mid_field_0")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("mid_field_1")), std::make_unique<role::BallAvoider>(role::BallAvoider("mid_field_2")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("offender_0")),  std::make_unique<role::BallAvoider>(role::BallAvoider("offender_1")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("offender_2"))};
}

uint8_t DefensiveStopFormation::score(world::World* world) noexcept { return 100; }

void DefensiveStopFormation::calculateInfoForRoles() noexcept {
    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter() + Vector2{0.5, 0.0});
    stpInfos["defender_0"].setPositionToMoveTo(Vector2{-length / 5, width / 3});
    stpInfos["defender_1"].setPositionToMoveTo(Vector2{-length / 5, -width / 3});
    stpInfos["defender_2"].setPositionToMoveTo(Vector2{-length / 5, width / 6});
    stpInfos["defender_3"].setPositionToMoveTo(Vector2{-length / 5, -width / 6});
    stpInfos["mid_field_0"].setPositionToMoveTo(Vector2{-length / 8, 0.0});
    stpInfos["mid_field_1"].setPositionToMoveTo(Vector2{-length / 9, -width / 4});
    stpInfos["mid_field_2"].setPositionToMoveTo(Vector2{-length / 9, width / 4});
    stpInfos["offender_0"].setPositionToMoveTo(Vector2{length / 4, 0.0});
    stpInfos["offender_1"].setPositionToMoveTo(Vector2{length / 4, width / 4});
    stpInfos["offender_2"].setPositionToMoveTo(Vector2{length / 4, -width / 4});
}

bool DefensiveStopFormation::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap DefensiveStopFormation::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::UNIQUE);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"defender_0", {notImportant}});
    flagMap.insert({"defender_1", {notImportant}});
    flagMap.insert({"defender_2", {notImportant}});
    flagMap.insert({"defender_3", {notImportant}});
    flagMap.insert({"mid_field_0", {notImportant}});
    flagMap.insert({"mid_field_1", {notImportant}});
    flagMap.insert({"mid_field_2", {notImportant}});
    flagMap.insert({"offender_0", {notImportant}});
    flagMap.insert({"offender_1", {notImportant}});
    flagMap.insert({"offender_2", {notImportant}});

    return flagMap;
}

const char* DefensiveStopFormation::getName() { return "Defensive Formation"; }
}  // namespace rtt::ai::stp::play