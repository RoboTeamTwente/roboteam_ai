//
// Created by jordi on 11-05-20.
//

#include "stp/roles/passive/Waller.h"
#include "stp/plays/contested/GetBallPossession.h"
#include "stp/computations/PositionComputations.h"
#include "stp/roles/active/BallGetter.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/Keeper.h"
#include "stp/evaluations/position/TimeToPositionEvaluation.h"

namespace rtt::ai::stp::play {

GetBallPossession::GetBallPossession() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallIsFree);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallClosestToUs);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallIsFree);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallClosestToUs);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::BallGetter>(role::BallGetter("ball_getter")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_0")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_1")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_0")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("waller_0")),
                                                                                 std::make_unique<role::Formation>(role::Formation("waller_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("waller_2"))};

    // initialize stpInfos
    stpInfos = std::unordered_map<std::string, StpInfo>{};
    for (auto &role : roles) {
        role->reset();
        auto roleName{role->getName()};
        stpInfos.emplace(roleName, StpInfo{});
    }
}

uint8_t GetBallPossession::score(PlayEvaluator& playEvaluator) noexcept {
    scoring = {{playEvaluator.getGlobalEvaluation(GlobalEvaluation::BallCloseToUs),1.0}};
               //std::make_pair(playEvaluator->getGlobalEvaluation(GlobalEvaluation::BallIsFree), 1)};
               //std::make_pair(stpInfos["ball_getter"].getRoleScore().value(),1)};
    return (lastScore = playEvaluator.calculateScore(scoring)).value();
}

void GetBallPossession::calculateInfoForScoredRoles(world::World* world) noexcept {
    //TODO-Jaro: When futureSTPInfo is a thing, make posToShootAt the receiver of a pass if that will be the next Play
    stpInfos["ball_getter"].setPositionToShootAt(field.getTheirGoalCenter());
    stpInfos["ball_getter"].setRoleScore(evaluation::TimeToPositionEvaluation().metricCheck(world->getWorld()->getRobotClosestToBall(world::us),
                                              world->getWorld()->getRobotClosestToBall(world::them), world->getWorld()->getBall().value()->getPos()));
}

void GetBallPossession::calculateInfoForRoles() noexcept {
    calculateInfoForScoredRoles(world);

    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    //If keeper has ball, shoot at one of our robots thats closest to our goal
    stpInfos["keeper"].setPositionToShootAt(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::us).value()->getPos());

    stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_0"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::them));
    stpInfos["defender_0"].setBlockDistance(BlockDistance::CLOSE);

    stpInfos["defender_1"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_1"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurBottomGoalSide(), world::them));
    stpInfos["defender_1"].setBlockDistance(BlockDistance::CLOSE);

    stpInfos["defender_2"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_2"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurTopGoalSide(), world::them));
    stpInfos["defender_2"].setBlockDistance(BlockDistance::CLOSE);

    stpInfos["midfielder_0"].setPositionToMoveTo(PositionComputations::getPosition(stpInfos["midfielder_0"].getPositionToMoveTo(),gen::gridMidFieldBot, gen::SafePosition, field, world));
    stpInfos["midfielder_1"].setPositionToMoveTo(PositionComputations::getPosition(stpInfos["midfielder_1"].getPositionToMoveTo(),gen::gridMidFieldMid, gen::SafePosition, field, world));
    stpInfos["midfielder_2"].setPositionToMoveTo(PositionComputations::getPosition(stpInfos["midfielder_2"].getPositionToMoveTo(),gen::gridMidFieldTop, gen::SafePosition, field, world));

    stpInfos["waller_0"].setPositionToMoveTo(PositionComputations::getWallPosition(0, 3, field, world));
    stpInfos["waller_1"].setPositionToMoveTo(PositionComputations::getWallPosition(1, 3, field, world));
    stpInfos["waller_2"].setPositionToMoveTo(PositionComputations::getWallPosition(2, 3, field, world));
}

Dealer::FlagMap GetBallPossession::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag ballGetterFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"ball_getter", {DealerFlagPriority::REQUIRED, {ballGetterFlag}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"midfielder_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"waller_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"waller_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

const char* GetBallPossession::getName() { return "Get Ball Possession"; }

    void GetBallPossession::storePlayInfo(gen::PlayInfos& info) noexcept {
        gen::StoreInfo ball_getter;
        ball_getter.robotID = stpInfos["ball_getter"].getRobot()->get()->getId();
        ball_getter.moveToPosition = stpInfos["ball_getter"].getPositionToMoveTo();
        info.insert({gen::KeyInfo::hasBall, ball_getter});
    }
}  // namespace rtt::ai::stp::play
