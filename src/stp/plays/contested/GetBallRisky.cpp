//
// Created by timovdk on 5/15/20.
//

#include "stp/plays/contested/GetBallRisky.h"
#include "stp/roles/active/BallGetter.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/PassReceiver.h"

namespace rtt::ai::stp::play {

    GetBallRisky::GetBallRisky() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
        startPlayEvaluation.emplace_back(eval::BallIsFree);
        startPlayEvaluation.emplace_back(eval::WeHaveMajority);
        startPlayEvaluation.emplace_back(eval::BallClosestToUs);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
        keepPlayEvaluation.emplace_back(eval::BallIsFree);
        keepPlayEvaluation.emplace_back(eval::WeHaveMajority);
        keepPlayEvaluation.emplace_back(eval::BallClosestToUs);

        roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
                std::make_unique<role::Keeper>(role::Keeper("keeper")),
                std::make_unique<role::BallGetter>(role::BallGetter("ball_getter")),
                std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_0")),
                std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_1")),
                std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_2")),
                std::make_unique<role::Defender>(role::Defender("defender_0")),
                std::make_unique<role::Defender>(role::Defender("defender_1")),
                std::make_unique<role::Defender>(role::Defender("defender_2")),
                std::make_unique<role::Defender>(role::Defender("midfielder_0")),
                std::make_unique<role::Defender>(role::Defender("midfielder_1")),
                std::make_unique<role::Defender>(role::Defender("midfielder_2"))};
    }

    uint8_t GetBallRisky::score(PlayEvaluator &playEvaluator) noexcept { return 120; }

    void GetBallRisky::calculateInfoForScoredRoles(world::World *world) noexcept {
        stpInfos["ball_getter"].setPositionToShootAt(world->getField().value().getTheirGoalCenter());
    }

    void GetBallRisky::calculateInfoForRoles() noexcept {
        calculateInfoForScoredRoles(world);

        auto enemyRobots = world->getWorld()->getThem();
        auto enemyAttacker = world->getWorld()->getRobotClosestToBall(world::them);

        if (enemyRobots.empty()) {
            RTT_ERROR("There are no enemy robots, which are necessary for this play!")
            return;
        }

        enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                         [&](const auto enemyRobot) -> bool {
                                             return enemyAttacker &&
                                                    enemyRobot->getId() ==
                                                    enemyAttacker.value()->getId();
                                         }));

        auto enemyClosestToGoal = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

        stpInfos["receiver_0"].setPositionToMoveTo(PositionComputations::getPosition(stpInfos["receiver_2"].getPositionToMoveTo(),gen::gridRightTop, gen::OffensivePosition, field, world));
        stpInfos["receiver_1"].setPositionToMoveTo(PositionComputations::getPosition(stpInfos["receiver_1"].getPositionToMoveTo(),gen::gridRightBot, gen::OffensivePosition, field, world));
        stpInfos["receiver_2"].setPositionToMoveTo(PositionComputations::getPosition(stpInfos["receiver_2"].getPositionToMoveTo(),gen::gridRightMid, gen::OffensivePosition, field, world));

        stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
        stpInfos["defender_0"].setEnemyRobot(enemyAttacker);
        stpInfos["defender_0"].setBlockDistance(BlockDistance::CLOSE);

        stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
        stpInfos["defender_1"].setEnemyRobot(enemyClosestToGoal);
        stpInfos["defender_1"].setBlockDistance(BlockDistance::CLOSE);

        if (enemyClosestToGoal) {
            stpInfos["defender_2"].setPositionToDefend(enemyClosestToGoal.value()->getPos());
        } else {
            stpInfos["defender_2"].setPositionToDefend(field.getOurGoalCenter() + Vector2{1, 1});
        }

        stpInfos["defender_2"].setEnemyRobot(enemyAttacker);
        stpInfos["defender_2"].setBlockDistance(BlockDistance::CLOSE);

        stpInfos["midfielder_0"].setPositionToDefend(field.getOurGoalCenter());
        stpInfos["midfielder_0"].setEnemyRobot(enemyAttacker);
        stpInfos["midfielder_0"].setBlockDistance(BlockDistance::CLOSE);

        stpInfos["midfielder_1"].setPositionToDefend(field.getOurGoalCenter());
        stpInfos["midfielder_1"].setEnemyRobot(enemyClosestToGoal);
        stpInfos["midfielder_1"].setBlockDistance(BlockDistance::CLOSE);

        if (enemyClosestToGoal)
            stpInfos["midfielder_2"].setPositionToDefend(enemyClosestToGoal.value()->getPos());
        else
            stpInfos["midfielder_2"].setPositionToDefend(field.getTheirGoalCenter() + Vector2{1, 1});
        stpInfos["midfielder_2"].setEnemyRobot(enemyAttacker);
        stpInfos["midfielder_2"].setBlockDistance(BlockDistance::CLOSE);
    }

    Dealer::FlagMap GetBallRisky::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag ballGetter(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
        flagMap.insert({"ball_getter", {DealerFlagPriority::REQUIRED, {ballGetter}}});
        flagMap.insert({"receiver_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"receiver_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"receiver_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"defender_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"defender_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"defender_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"midfielder_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"midfielder_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
        flagMap.insert({"midfielder_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
        return flagMap;
    }

    const char *GetBallRisky::getName() { return "Get Ball Risky"; }

}  // namespace rtt::ai::stp::play
