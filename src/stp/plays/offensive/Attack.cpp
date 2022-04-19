//
// Created by jordi on 24-03-20.
/// TODO-Max change to take ShootAtGoal
//

#include "stp/plays/offensive/Attack.h"

#include "stp/computations/GoalComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Attacker.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

Attack::Attack() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);
    startPlayEvaluation.emplace_back(eval::TheyDoNotHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);
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
                                                                                       std::make_unique<role::Defender>(("defender_left")),
                                                                                       std::make_unique<role::Defender>(("defender_mid")),
                                                                                       std::make_unique<role::Defender>(("defender_right"))};
}

uint8_t Attack::score(PlayEvaluator& playEvaluator) noexcept {
    auto world = playEvaluator.getWorld();
    auto field = world->getField().value();

    // Score the position of the ball based on the odds of scoring
    return PositionScoring::scorePosition(world->getWorld()->getBall().value()->getPos(), gen::GoalShot, field, world).score;
}

Dealer::FlagMap Attack::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag attackerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"striker", {DealerFlagPriority::REQUIRED, {}}});
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

void Attack::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    /// TODO let keeper shot at a good position
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    stpInfos["attacker"].setPositionToShootAt(goalTarget);
    stpInfos["attacker"].setKickOrChip(KickOrChip::KICK);
    stpInfos["attacker"].setShotType(ShotType::MAX);

    // Set the midfielders to go to the part of the field where the ball is NOT (in y-direction)
    if (world->getWorld()->getBall().value()->getPos().y > field.getFrontLeftGrid().getOffSetY()) {  // Ball is in left of field
        stpInfos["midfielder_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::OffensivePosition, field, world));
        stpInfos["midfielder_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleRightGrid(), gen::OffensivePosition, field, world));
    } else if (world->getWorld()->getBall().value()->getPos().y < field.getMiddleMidGrid().getOffSetY()) {  // Ball is in right of field
        stpInfos["midfielder_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::OffensivePosition, field, world));
        stpInfos["midfielder_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::OffensivePosition, field, world));
    } else {  // Ball is in middle of field
        stpInfos["midfielder_1"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::OffensivePosition, field, world));
        stpInfos["midfielder_2"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleRightGrid(), gen::OffensivePosition, field, world));
    }
}

bool Attack::shouldEndPlay() noexcept {
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) { return role != nullptr && role->getName() == "attacker" && role->finished(); });
}

const char* Attack::getName() { return "Attack"; }

}  // namespace rtt::ai::stp::play
