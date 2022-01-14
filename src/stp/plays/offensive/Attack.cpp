//
// Created by jordi on 24-03-20.
/// TODO-Max change to take ShootAtGoal
//

#include "stp/plays/offensive/Attack.h"

#include "stp/computations/GoalComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Attacker.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

Attack::Attack() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);
    startPlayEvaluation.emplace_back(eval::BallClosestToUs);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);
    keepPlayEvaluation.emplace_back(eval::BallCloseToUs);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")), std::make_unique<role::Attacker>(role::Attacker("attacker")),
        std::make_unique<role::Formation>(role::Formation("midfielder_1")), std::make_unique<role::Formation>(role::Formation("midfielder_2"))};
}

uint8_t Attack::score(PlayEvaluator& playEvaluator) noexcept {
    if (playEvaluator.getWorld()->getWorld()->getBall().value()->getPos().dist(field.getTheirGoalCenter()) < field.getFieldLength() / 2) {
        return 150;
    } else
        return 60;
}

Dealer::FlagMap Attack::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag attackerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"attacker", {DealerFlagPriority::REQUIRED, {attackerFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void Attack::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    /// TODO let keeper shot at a good position
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    stpInfos["attacker"].setPositionToShootAt(goalTarget);
    stpInfos["attacker"].setShotType(ShotType::PASS);

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

    // Midfielders
    stpInfos["midfielder_1"].setPositionToMoveTo(Vector2(0.0, field.getFieldWidth() / 4));
    stpInfos["midfielder_2"].setPositionToMoveTo(Vector2(0.0, -field.getFieldWidth() / 4));
    stpInfos["midfielder_3"].setPositionToMoveTo(Vector2(field.getFieldLength() / 8, 0.0));
    stpInfos["midfielder_4"].setPositionToMoveTo(Vector2(-field.getFieldLength() / 8, 0.0));

    // Defenders
    stpInfos["defender_1"].setPositionToMoveTo(Vector2(-length / 4, width / 8));
    stpInfos["defender_2"].setPositionToMoveTo(Vector2(-length / 4, -width / 8));
    stpInfos["defender_3"].setPositionToMoveTo(Vector2(-length / 4.5, width / 3));

    //    stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
    //    stpInfos["defender_1"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::them));
    //    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);
    //
    //    stpInfos["defender_2"].setPositionToDefend(field.getOurTopGoalSide());
    //    stpInfos["defender_2"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurTopGoalSide(), world::them));
    //    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);
    //
    //    stpInfos["defender_3"].setPositionToDefend(field.getOurBottomGoalSide());
    //    stpInfos["defender_3"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurBottomGoalSide(), world::them));
    //    stpInfos["defender_3"].setBlockDistance(BlockDistance::HALFWAY);
}

const char* Attack::getName() { return "Attack"; }

}  // namespace rtt::ai::stp::play
