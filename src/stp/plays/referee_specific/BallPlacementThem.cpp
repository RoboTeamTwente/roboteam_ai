//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/BallPlacementThem.h"
#include "stp/roles/passive/BallAvoider.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

    BallPlacementThem::BallPlacementThem() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(eval::BallPlacementThemGameState);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(eval::BallPlacementThemGameState);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Keeper>(role::Keeper("keeper")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_0")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_1")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_2")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_3")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_4")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_5")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_6")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_7")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_8")),
                std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_9"))};
    }

    uint8_t BallPlacementThem::score(PlayEvaluator &playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::BallPlacementThemGameState), 1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

    void BallPlacementThem::calculateInfoForRoles() noexcept {
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

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

    Dealer::FlagMap BallPlacementThem::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;

        flagMap.insert({"keeper", {Priority::KEEPER, {}}});
        flagMap.insert({"ball_avoider_0", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_1", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_2", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_3", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_4", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_5", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_6", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_7", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_8", {Priority::LOW_PRIORITY, {}}});
        flagMap.insert({"ball_avoider_9", {Priority::LOW_PRIORITY, {}}});

        return flagMap;
    }

    const char *BallPlacementThem::getName() { return "Ball Placement Them"; }
}  // namespace rtt::ai::stp::play
