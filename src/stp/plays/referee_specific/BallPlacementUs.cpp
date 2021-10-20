//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/BallPlacementUs.h"
#include "stp/roles/passive/BallAvoider.h"
#include "stp/roles/active/BallPlacer.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::play {

    BallPlacementUs::BallPlacementUs() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(eval::BallPlacementUsGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(eval::BallPlacementUsGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::BallPlacer>(role::BallPlacer("ball_placer")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_1")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_2")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_3")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_4")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_5")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_6")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_7")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_8")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_9")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("keeper"))};
    }

    uint8_t BallPlacementUs::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::BallPlacementUsGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    void BallPlacementUs::calculateInfoForRoles() noexcept {
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

        auto ballTarget = rtt::ai::GameStateManager::getRefereeDesignatedPosition();
        stpInfos["ball_placer"].setPositionToMoveTo(ballTarget);

        if(stpInfos["ball_placer"].getRobot()->get() && ballTarget.length() > 0 && (stpInfos["ball_placer"].getRobot()->get()->getPos() - ballTarget).length() < 0.55) {
            stpInfos["ball_placer"].setDribblerSpeed(0);
            stpInfos["ball_placer"].setPositionToMoveTo(stpInfos["ball_placer"].getRobot()->get()->getPos());
        }

        auto length = field.getFieldLength();
        auto width = field.getFieldWidth();

        stpInfos["ball_avoider_1"].setPositionToMoveTo(Vector2{-length / 5, 0.0});
        stpInfos["ball_avoider_2"].setPositionToMoveTo(Vector2{-length / 5, width / 6});
        stpInfos["ball_avoider_3"].setPositionToMoveTo(Vector2{length / 5, -width / 6});
        stpInfos["ball_avoider_4"].setPositionToMoveTo(Vector2{-length / 8, 0.0});
        stpInfos["ball_avoider_5"].setPositionToMoveTo(Vector2{-length / 9, -width / 4});
        stpInfos["ball_avoider_6"].setPositionToMoveTo(Vector2{length / 9, width / 4});
        stpInfos["ball_avoider_7"].setPositionToMoveTo(Vector2{length / 4, 0.0});
        stpInfos["ball_avoider_8"].setPositionToMoveTo(Vector2{length / 4, width / 4});
        stpInfos["ball_avoider_9"].setPositionToMoveTo(Vector2{length / 4, -width / 4});
    }

    Dealer::FlagMap BallPlacementUs::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag ballPlacement(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::REQUIRED);

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
        flagMap.insert({"ball_placer", {DealerFlagPriority::REQUIRED, {ballPlacement}}});
        flagMap.insert({"ball_avoider_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

        return flagMap;
    }

    const char *BallPlacementUs::getName() { return "Ball Placement Us"; }
}  // namespace rtt::ai::stp::play
