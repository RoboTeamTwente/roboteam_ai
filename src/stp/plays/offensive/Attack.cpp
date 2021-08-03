//
// Created by jordi on 24-03-20.
/// TODO-Max change to take ShootAtGoal
//

#include "stp/plays/offensive/Attack.h"
#include "stp/computations/GoalComputations.h"
#include "stp/roles/active/Attacker.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

Attack::Attack() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);
    startPlayEvaluation.emplace_back(eval::BallClosestToUs);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);
    keepPlayEvaluation.emplace_back(eval::BallCloseToUs);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::Attacker>(role::Attacker("attacker")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_3")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_4")),
                                                                                 std::make_unique<role::Formation>(role::Formation("defender_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("defender_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("defender_3"))};
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
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"attacker", {DealerFlagPriority::REQUIRED, {attackerFlag}}});
    flagMap.insert({"offender_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToTheirGoalFlag}}});
    flagMap.insert({"offender_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToTheirGoalFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_2", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_3", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_4", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});

    return flagMap;
}

void Attack::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    // Attacker
    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    stpInfos["attacker"].setPositionToShootAt(goalTarget);
    stpInfos["attacker"].setShotType(ShotType::PASS);

    // Offenders
    stpInfos["offender_1"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, field.getFieldWidth() / 4));
    stpInfos["offender_2"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, -field.getFieldWidth() / 4));

    // Midfielders
    stpInfos["midfielder_1"].setPositionToMoveTo(Vector2(0.0, field.getFieldWidth() / 4));
    stpInfos["midfielder_2"].setPositionToMoveTo(Vector2(0.0, -field.getFieldWidth() / 4));
    stpInfos["midfielder_3"].setPositionToMoveTo(Vector2(field.getFieldLength() / 8, 0.0));
    stpInfos["midfielder_4"].setPositionToMoveTo(Vector2(-field.getFieldLength() / 8, 0.0));

    // Defenders
    stpInfos["defender_1"].setPositionToMoveTo(Vector2(-length / 4, width/8));
    stpInfos["defender_2"].setPositionToMoveTo(Vector2(-length / 4, -width/8));
    stpInfos["defender_3"].setPositionToMoveTo(Vector2(-length / 4.5, width/3));

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

const char *Attack::getName() { return "Attack"; }

}  // namespace rtt::ai::stp::play
