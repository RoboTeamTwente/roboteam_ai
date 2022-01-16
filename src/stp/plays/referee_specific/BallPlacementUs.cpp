//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/BallPlacementUs.h"

#include "stp/roles/active/BallPlacer.h"
#include "stp/roles/passive/BallAvoider.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::play {

BallPlacementUs::BallPlacementUs() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::BallPlacementUsGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::BallPlacementUsGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::BallPlacer>(role::BallPlacer("ball_placer")), std::make_unique<role::BallAvoider>(role::BallAvoider("keeper")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_0")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_1"))};
}

uint8_t BallPlacementUs::score(PlayEvaluator &playEvaluator) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{playEvaluator.getGlobalEvaluation(eval::BallPlacementUsGameState), 1.0}};
    return (lastScore = playEvaluator.calculateScore(scoring)).value();  // DONT TOUCH.
}

void BallPlacementUs::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    auto ballTarget = rtt::ai::GameStateManager::getRefereeDesignatedPosition();

    // Adjust placement position to be one robot radius away in the distance of movement
    if (stpInfos["ball_placer"].getRobot()) ballTarget -= (ballTarget - stpInfos["ball_placer"].getRobot()->get()->getPos()).stretchToLength(control_constants::ROBOT_RADIUS);

    stpInfos["ball_placer"].setPositionToMoveTo(ballTarget);

    if (stpInfos["ball_placer"].getRobot() && stpInfos["ball_placer"].getRobot()->get()->getDistanceToBall() < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        stpInfos["ball_placer"].setDribblerSpeed(100);
    }
}

Dealer::FlagMap BallPlacementUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag ballPlacement(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"ball_placer", {DealerFlagPriority::REQUIRED, {ballPlacement}}});
    flagMap.insert({"ball_avoider_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_1", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

const char *BallPlacementUs::getName() { return "Ball Placement Us"; }
}  // namespace rtt::ai::stp::play
