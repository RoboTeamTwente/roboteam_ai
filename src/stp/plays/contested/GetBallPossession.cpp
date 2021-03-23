//
// Created by jordi on 11-05-20.
//

#include <stp/roles/passive/Waller.h>
#include "include/roboteam_ai/stp/plays/contested/GetBallPossession.h"
#include "include/roboteam_ai/stp/computations/PositionComputations.h"
#include "include/roboteam_ai/stp/roles/active/BallGetter.h"
#include "include/roboteam_ai/stp/roles/passive/Defender.h"
#include "include/roboteam_ai/stp/roles/passive/Formation.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

GetBallPossession::GetBallPossession() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
//    startPlayEvaluation.emplace_back(GlobalEvaluation::BallIsFree);
//    startPlayEvaluation.emplace_back(GlobalEvaluation::BallClosestToUs);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallOnOurSide);
//    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallClosestToUs);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::BallGetter>(role::BallGetter("ball_getter")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_0")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_1")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_0")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_0")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_2"))};

    // initialize stpInfos
    stpInfos = std::unordered_map<std::string, StpInfo>{};
    for (auto &role : roles) {
        role->reset();
        auto roleName{role->getName()};
        stpInfos.emplace(roleName, StpInfo{});
    }
}

uint8_t GetBallPossession::score(PlayEvaluator& playEvaluator) noexcept {
    calculateInfoForScoredRoles(playEvaluator.getWorld());
    scoring = {{playEvaluator.getGlobalEvaluation(GlobalEvaluation::BallCloseToUs),1.0}};
               //std::make_pair(playEvaluator->getGlobalEvaluation(GlobalEvaluation::BallIsFree), 1)};
               //std::make_pair(stpInfos["ball_getter"].getRoleScore().value(),1)};
    return (lastScore = playEvaluator.calculateScore(scoring)).value();
}

void GetBallPossession::calculateInfoForScoredRoles(world::World* world) noexcept {
    //TODO-Jaro: Find out why GetBallPossession has a shootPos, and remove/improve if necessary
    stpInfos["ball_getter"].setPositionToShootAt(Vector2{0, 0});
    stpInfos["ball_getter"].setRoleScore(100);
}

void GetBallPossession::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    //TODO-Jaro: Find out why GetBallPossession has a shootPos, and remove/improve if necessary
    stpInfos["ball_getter"].setPositionToShootAt(field.getTheirGoalCenter());

    stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_0"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::them));
    stpInfos["defender_0"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_1"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_1"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurBottomGoalSide(), world::them));
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_2"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurTopGoalSide(), world::them));
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["midfielder_0"].setPositionToMoveTo(Vector2(0.0, width / 4));
    stpInfos["midfielder_1"].setPositionToMoveTo(Vector2(0.0, -width / 4));
    stpInfos["midfielder_2"].setPositionToMoveTo(Vector2(-length / 8, 0.0));

    int amountDefenders = 3;
    std::vector<Vector2> wallPositions = {};
    if(FieldComputations::pointIsValidPosition(field, world->getWorld().value().getBall().value()->getPos()))
        wallPositions = computations::PositionComputations::determineWallPositions(field,world,amountDefenders);
    if (!wallPositions.empty()) {
        stpInfos["waller_0"].setPositionToMoveTo(wallPositions.at(0));
        stpInfos["waller_1"].setPositionToMoveTo(wallPositions.at(1));
        stpInfos["waller_2"].setPositionToMoveTo(wallPositions.at(2));
    } else {
        stpInfos["waller_0"].setPositionToMoveTo(Vector2(0,0));
        stpInfos["waller_1"].setPositionToMoveTo(Vector2(0,0.2));
        stpInfos["waller_2"].setPositionToMoveTo(Vector2(0,-0.2));
    }
}

bool GetBallPossession::shouldRoleSkipEndTactic() { return false; }

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

    std::vector<std::string> GetBallPossession::getScoredRoles() {
        return {"ball_getter"};
    }
}  // namespace rtt::ai::stp::play
