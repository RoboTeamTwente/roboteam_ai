//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/BallPlacementThem.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallAvoider.h"

namespace rtt::ai::stp::play {

BallPlacementThem::BallPlacementThem() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::BallPlacementThemGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::BallPlacementThemGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::BallAvoider>(role::BallAvoider("keeper")),         std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_0")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_1")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_2")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_3")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_4")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_5")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_6")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_7")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_8")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_9"))};
}

uint8_t BallPlacementThem::score(const rtt::world::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::BallPlacementThemGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

void BallPlacementThem::calculateInfoForRoles() noexcept {}

Dealer::FlagMap BallPlacementThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"ball_avoider_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_9", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

const char* BallPlacementThem::getName() { return "Ball Placement Them"; }
}  // namespace rtt::ai::stp::play
