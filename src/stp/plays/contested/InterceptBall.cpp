//
// Created by Alexander on 11-05-2022
//
#include "stp/plays/contested/InterceptBall.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Attacker.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/active/BallInterceptor.h"


namespace rtt::ai::stp::play {

InterceptBall::InterceptBall() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::BallIsFree);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::BallIsFree);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(("keeper")),
                                                                                       std::make_unique<role::BallInterceptor>(("interceptor_primary")),
                                                                                       std::make_unique<role::BallInterceptor>(("interceptor_secondary")),
                                                                                       std::make_unique<role::Formation>(("attacker_1")),
                                                                                       std::make_unique<role::Formation>(("attacker_2")),
                                                                                       std::make_unique<role::Formation>(("midfielder_left")),
                                                                                       std::make_unique<role::Formation>(("midfielder_mid")),
                                                                                       std::make_unique<role::Formation>(("midfielder_right")),
                                                                                       std::make_unique<role::BallDefender>(("defender_left")),
                                                                                       std::make_unique<role::BallDefender>(("defender_mid")),
                                                                                       std::make_unique<role::BallDefender>(("defender_right"))};
}

uint8_t InterceptBall::score(const rtt::world::Field& field) noexcept {
    return world->getWorld()->getBall()->get()->velocity.length() > control_constants::BALL_STILL_VEL ? 255 : 0;
}

Dealer::FlagMap InterceptBall::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag interceptorFlag(DealerFlagTitle::READY_TO_INTERCEPT_GOAL_SHOT, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"interceptor_primary", {DealerFlagPriority::REQUIRED, {interceptorFlag}}});
    flagMap.insert({"interceptor_secondary", {DealerFlagPriority::REQUIRED, {interceptorFlag}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_mid", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"midfielder_right", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_left", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_mid", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_right", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void InterceptBall::calculateInfoForRoles() noexcept {
    calculateInfoForAttackers();
    calculateInfoForMidfielders();
    calculateInfoForDefenders();

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
}

void InterceptBall::calculateInfoForDefenders() noexcept {
    stpInfos["defender_left"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_left"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_mid"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_mid"].setBlockDistance(BlockDistance::CLOSE);

    stpInfos["defender_right"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_right"].setBlockDistance(BlockDistance::HALFWAY);
}

void InterceptBall::calculateInfoForMidfielders() noexcept {
    stpInfos["midfielder_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::OffensivePosition, field, world));
    stpInfos["midfielder_mid"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::BlockingPosition, field, world));
    stpInfos["midfielder_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleRightGrid(), gen::OffensivePosition, field, world));
}

void InterceptBall::calculateInfoForAttackers() noexcept {
    // Set the attackers to go to the part of the field where the ball is NOT (in y-direction), since that is where the interceptor(s) will go
    if (world->getWorld()->getBall().value()->position.y > field.getFrontLeftGrid().getOffSetY()) {  // Ball is in left of field
        stpInfos["attacker_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontMidGrid(), gen::OffensivePosition, field, world));
        stpInfos["attacker_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
    } else if (world->getWorld()->getBall().value()->position.y < field.getFrontMidGrid().getOffSetY()) {  // Ball is in right of field
        stpInfos["attacker_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
        stpInfos["attacker_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontMidGrid(), gen::OffensivePosition, field, world));
    } else {  // Ball is in middle of field
        stpInfos["attacker_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
        stpInfos["attacker_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
    }
}

bool InterceptBall::shouldEndPlay() noexcept {
    return world->getWorld()->getBall()->get()->velocity.length() < control_constants::BALL_STILL_VEL;
}

const char* InterceptBall::getName() { return "InterceptBall"; }

}  // namespace rtt::ai::stp::play
