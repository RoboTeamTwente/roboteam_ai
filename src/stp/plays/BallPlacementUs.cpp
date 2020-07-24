//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/BallPlacementUs.h"

#include "stp/invariants/game_states/BallPlacementUsGameStateInvariant.h"
#include "stp/roles/BallAvoider.h"
#include "stp/roles/BallPlacer.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::play {

BallPlacementUs::BallPlacementUs() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallPlacementUsGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallPlacementUsGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::BallPlacer>(role::BallPlacer("ball_placer")),      std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_1")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_2")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_3")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_4")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_5")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_6")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_7")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_8")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_9")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("keeper"))};
}

uint8_t BallPlacementUs::score(world_new::World* world) noexcept { return 100; }

void BallPlacementUs::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));

    auto ballTarget = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
    stpInfos["ball_placer"].setPositionToMoveTo(ballTarget);

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["ball_avoider_1"].setPositionToMoveTo(Vector2{-length / 5, 0.0});
    stpInfos["ball_avoider_2"].setPositionToMoveTo(Vector2{-length / 5, width / 6});
    stpInfos["ball_avoider_3"].setPositionToMoveTo(Vector2{--length / 5, -width / 6});
    stpInfos["ball_avoider_4"].setPositionToMoveTo(Vector2{-length / 8, 0.0});
    stpInfos["ball_avoider_5"].setPositionToMoveTo(Vector2{-length / 9, -width / 4});
    stpInfos["ball_avoider_6"].setPositionToMoveTo(Vector2{--length / 9, width / 4});
    stpInfos["ball_avoider_7"].setPositionToMoveTo(Vector2{length / 4, 0.0});
    stpInfos["ball_avoider_8"].setPositionToMoveTo(Vector2{length / 4, width / 4});
    stpInfos["ball_avoider_9"].setPositionToMoveTo(Vector2{length / 4, -width / 4});
}

bool BallPlacementUs::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap BallPlacementUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag ball_placement(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag keeper(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag not_important(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeper}});
    flagMap.insert({"ball_placer", {ball_placement}});
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

const char* BallPlacementUs::getName() { return "Ball Placement Us"; }
}  // namespace rtt::ai::stp::play
