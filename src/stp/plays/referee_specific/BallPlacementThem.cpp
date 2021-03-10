//
// Created by jessevw on 24.03.20.
//

#include "include/roboteam_ai/stp/plays/referee_specific/BallPlacementThem.h"

#include "stp/invariants/game_states/BallPlacementThemGameStateEvaluation.h"
#include "include/roboteam_ai/stp/roles/passive/BallAvoider.h"

namespace rtt::ai::stp::play {

BallPlacementThem::BallPlacementThem() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallPlacementThemGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallPlacementThemGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::BallAvoider>(role::BallAvoider("keeper")),         std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_0")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_1")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_2")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_3")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_4")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_5")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_6")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_7")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_8")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_9"))};
}

uint8_t BallPlacementThem::score(world::World* world) noexcept { return 0; }

void BallPlacementThem::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["ball_avoider_0"].setPositionToMoveTo(Vector2{-length / 5, -width / 3});
    stpInfos["ball_avoider_1"].setPositionToMoveTo(Vector2{-length / 5, width / 3});
    stpInfos["ball_avoider_2"].setPositionToMoveTo(Vector2{-length / 5, width / 6});
    stpInfos["ball_avoider_3"].setPositionToMoveTo(Vector2{--length / 5, -width / 6});
    stpInfos["ball_avoider_4"].setPositionToMoveTo(Vector2{-length / 8, 0.0});
    stpInfos["ball_avoider_5"].setPositionToMoveTo(Vector2{-length / 9, -width / 4});
    stpInfos["ball_avoider_6"].setPositionToMoveTo(Vector2{--length / 9, width / 4});
    stpInfos["ball_avoider_7"].setPositionToMoveTo(Vector2{length / 4, 0.0});
    stpInfos["ball_avoider_8"].setPositionToMoveTo(Vector2{length / 4, width / 4});
    stpInfos["ball_avoider_9"].setPositionToMoveTo(Vector2{length / 4, -width / 4});
}

bool BallPlacementThem::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap BallPlacementThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::UNIQUE);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"ball_avoider_0", {notImportant}});
    flagMap.insert({"ball_avoider_1", {notImportant}});
    flagMap.insert({"ball_avoider_2", {notImportant}});
    flagMap.insert({"ball_avoider_3", {notImportant}});
    flagMap.insert({"ball_avoider_4", {notImportant}});
    flagMap.insert({"ball_avoider_5", {notImportant}});
    flagMap.insert({"ball_avoider_6", {notImportant}});
    flagMap.insert({"ball_avoider_7", {notImportant}});
    flagMap.insert({"ball_avoider_8", {notImportant}});
    flagMap.insert({"ball_avoider_9", {notImportant}});

    return flagMap;
}

const char* BallPlacementThem::getName() { return "Ball Placement Them"; }
}  // namespace rtt::ai::stp::play
