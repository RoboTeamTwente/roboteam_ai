//
// Created by jessevw on 24.03.20.
//

#include "stp/new_plays/BallPlacementThem.h"

#include "stp/invariants/game_states/BallPlacementThemGameStateInvariant.h"
#include "stp/new_roles/BallAvoider.h"

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

uint8_t BallPlacementThem::score(world_new::World* world) noexcept { return 0; }

void BallPlacementThem::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));

    stpInfos["ball_avoider_0"].setPositionToMoveTo(Vector2{-1, -4});
    stpInfos["ball_avoider_1"].setPositionToMoveTo(Vector2{-3, 4});
    stpInfos["ball_avoider_2"].setPositionToMoveTo(Vector2{-3, 1});
    stpInfos["ball_avoider_3"].setPositionToMoveTo(Vector2{-3, -1});
    stpInfos["ball_avoider_4"].setPositionToMoveTo(Vector2{-3, -4});
    stpInfos["ball_avoider_5"].setPositionToMoveTo(Vector2{-2, 3});
    stpInfos["ball_avoider_6"].setPositionToMoveTo(Vector2{-2, 0});
    stpInfos["ball_avoider_7"].setPositionToMoveTo(Vector2{-2, -3});
    stpInfos["ball_avoider_8"].setPositionToMoveTo(Vector2{-1, 4});
    stpInfos["ball_avoider_9"].setPositionToMoveTo(Vector2{-1, 0});
}

bool BallPlacementThem::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap BallPlacementThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeper(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeper}});
    flagMap.insert({"ball_avoider_0", {not_important}});
    flagMap.insert({"ball_avoider_1", {not_important}});
    flagMap.insert({"ball_avoider_2", {not_important}});
    flagMap.insert({"ball_avoider_3", {not_important}});
    flagMap.insert({"ball_avoider_4", {not_important}});
    flagMap.insert({"ball_avoider_5", {not_important}});
    flagMap.insert({"ball_avoider_6", {not_important}});
    flagMap.insert({"ball_avoider_7", {not_important}});
    flagMap.insert({"ball_avoider_8", {not_important}});
    flagMap.insert({"ball_avoider_9", {not_important}});

    return flagMap;
}

const char* BallPlacementThem::getName() { return "Ball Placement Them"; }
}  // namespace rtt::ai::stp::play
