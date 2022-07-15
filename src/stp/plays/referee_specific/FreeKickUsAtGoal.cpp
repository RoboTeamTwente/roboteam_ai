//
// Created by Floris Hoek on 22-06-21.
//

#include "stp/plays/referee_specific/FreeKickUsAtGoal.h"

#include "stp/computations/GoalComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/FreeKickTaker.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

FreeKickUsAtGoal::FreeKickUsAtGoal() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::FreeKickUsGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(("keeper")),
                                                                                       std::make_unique<role::FreeKickTaker>(("free_kick_taker")),
                                                                                       std::make_unique<role::Formation>(("attacker_1")),
                                                                                       std::make_unique<role::Formation>(("attacker_2")),
                                                                                       std::make_unique<role::Formation>(("midfielder_left")),
                                                                                       std::make_unique<role::Formation>(("midfielder_mid")),
                                                                                       std::make_unique<role::Formation>(("midfielder_right")),
                                                                                       std::make_unique<role::Formation>(("attacking_midfielder")),
                                                                                       std::make_unique<role::BallDefender>(("defender_left")),
                                                                                       std::make_unique<role::BallDefender>(("defender_mid")),
                                                                                       std::make_unique<role::BallDefender>(("defender_right"))};
}

uint8_t FreeKickUsAtGoal::score(const rtt::world::Field& field) noexcept {
    // If we are in the FreeKickUsAtGoal gameState, we always want to execute this play
    return PositionScoring::scorePosition(world->getWorld()->getBall().value()->position, gen::GoalShot, field, world).score;
}

Dealer::FlagMap FreeKickUsAtGoal::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);

    Dealer::DealerFlag freeKickTakerFirstPriority(DealerFlagTitle::CAN_KICK_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag freeKickTakerSecondPriority(DealerFlagTitle::CAN_DETECT_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag freeKickTakerThirdPriority(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"free_kick_taker", {DealerFlagPriority::REQUIRED, {freeKickTakerFirstPriority, freeKickTakerSecondPriority, freeKickTakerThirdPriority}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"midfielder_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_mid", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_right", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacking_midfielder", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_mid", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_right", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void FreeKickUsAtGoal::calculateInfoForRoles() noexcept {
    calculateInfoForAttackers();
    calculateInfoForMidfielders();
    calculateInfoForDefenders();

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    // FreeKickTaker
    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    stpInfos["free_kick_taker"].setPositionToShootAt(goalTarget);
    stpInfos["free_kick_taker"].setKickOrChip(KickOrChip::KICK);
    stpInfos["free_kick_taker"].setShotType(ShotType::MAX);
}

void FreeKickUsAtGoal::calculateInfoForDefenders() noexcept {
    stpInfos["defender_left"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_left"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_mid"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_mid"].setBlockDistance(BlockDistance::CLOSE);

    stpInfos["defender_right"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_right"].setBlockDistance(BlockDistance::HALFWAY);
}

void FreeKickUsAtGoal::calculateInfoForMidfielders() noexcept {
    stpInfos["midfielder_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::OffensivePosition, field, world));
    stpInfos["midfielder_mid"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::BlockingPosition, field, world));
    stpInfos["midfielder_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleRightGrid(), gen::OffensivePosition, field, world));

    // If the ball (and therefore striker) are in the front of the field, let the attacking midfielder go to the midfield
    // If the striker is not in the front field already, let the attacking midfielder go to the free section in the front field
    if (world->getWorld()->getBall()->get()->position.x > field.getFrontMidGrid().getOffSetX()) {
        stpInfos["attacking_midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::OffensivePosition, field, world));
    } else {
        if (world->getWorld()->getBall().value()->position.y > field.getFrontLeftGrid().getOffSetY()) {  // Ball is in left of field
            stpInfos["attacking_midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
        } else if (world->getWorld()->getBall().value()->position.y < field.getFrontMidGrid().getOffSetY()) {  // Ball is in right of field
            stpInfos["attacking_midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
        } else {  // Ball is in middle of field
            stpInfos["attacking_midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontMidGrid(), gen::OffensivePosition, field, world));
        }
    }
}

void FreeKickUsAtGoal::calculateInfoForAttackers() noexcept {
    // Set the attackers to go to the part of the field where the ball is NOT (in y-direction), since that is where the striker will be
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

bool FreeKickUsAtGoal::shouldEndPlay() noexcept {
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "free_kick_taker" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

const char* FreeKickUsAtGoal::getName() { return "Free Kick Us At Goal"; }

}  // namespace rtt::ai::stp::play
