//
// Created by jordi on 24-03-20.
/// TODO-Max change to take ShootAtGoal
//

#include "stp/plays/offensive/Attack.h"

#include "stp/computations/GoalComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Attacker.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

Attack::Attack() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::TheyDoNotHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::TheyDoNotHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(("keeper")),
                                                                                       std::make_unique<role::Attacker>(("striker")),
                                                                                       std::make_unique<role::Formation>(("attacker_1")),
                                                                                       std::make_unique<role::Formation>(("attacker_2")),
                                                                                       std::make_unique<role::Formation>(("midfielder_left")),
                                                                                       std::make_unique<role::Formation>(("midfielder_mid")),
                                                                                       std::make_unique<role::Formation>(("midfielder_right")),
                                                                                       std::make_unique<role::Formation>(("attacking_midfielder")),
                                                                                       std::make_unique<role::BallDefender>(("defender_left")),
                                                                                       std::make_unique<role::Formation>(("defender_mid")),
                                                                                       std::make_unique<role::BallDefender>(("defender_right"))};
}

uint8_t Attack::score(const rtt::world::Field& field) noexcept {
    // Score the position of the ball based on the odds of scoring
    return PositionScoring::scorePosition(world->getWorld()->getBall().value()->position, gen::GoalShot, field, world).score;
}

Dealer::FlagMap Attack::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag kickerFirstPriority(DealerFlagTitle::CAN_KICK_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag kickerSecondPriority(DealerFlagTitle::CAN_DETECT_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag kickerThirdPriority(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"striker", {DealerFlagPriority::REQUIRED, {kickerFirstPriority, kickerSecondPriority, kickerThirdPriority}}});
    flagMap.insert({"attacker_1", {DealerFlagPriority::HIGH_PRIORITY, {kickerFirstPriority, kickerSecondPriority}}});
    flagMap.insert({"attacker_2", {DealerFlagPriority::HIGH_PRIORITY, {kickerFirstPriority, kickerSecondPriority}}});
    flagMap.insert({"midfielder_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_mid", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_right", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacking_midfielder", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_mid", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_right", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void Attack::calculateInfoForRoles() noexcept {
    calculateInfoForAttackers();
    calculateInfoForMidfielders();
    calculateInfoForDefenders();

    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    // Striker
    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    goalTarget.y = std::clamp(goalTarget.y, field.getTheirBottomGoalSide().y + 0.4, field.getTheirTopGoalSide().y - 0.4);
    stpInfos["striker"].setPositionToShootAt(goalTarget);
    stpInfos["striker"].setKickOrChip(KickOrChip::KICK);
    stpInfos["striker"].setShotType(ShotType::MAX);
}

void Attack::calculateInfoForDefenders() noexcept {
    stpInfos["defender_left"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_left"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_mid"].setPositionToMoveTo(PositionComputations::getBallBlockPosition(field, world));
    stpInfos["defender_mid"].setAngle((world->getWorld()->getBall().value()->position - stpInfos["defender_mid"].getPositionToMoveTo().value()).toAngle());

    stpInfos["defender_right"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_right"].setBlockDistance(BlockDistance::HALFWAY);
}

void Attack::calculateInfoForMidfielders() noexcept {
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

void Attack::calculateInfoForAttackers() noexcept {
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

bool Attack::shouldEndPlay() noexcept {
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) { return role != nullptr && role->getName() == "striker" && role->finished(); });
}

const char* Attack::getName() { return "Attack"; }

}  // namespace rtt::ai::stp::play
