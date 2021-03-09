//
// Created by jordi on 24-03-20.
/// TODO-Max change to take ShootAtGoal
//

#include "include/roboteam_ai/stp/plays/offensive/Attack.h"
#include "include/roboteam_ai/stp/computations/GoalComputations.h"
#include "stp/invariants/BallCloseToUsGlobalEvaluation.h"
#include "stp/invariants/BallClosestToUsGlobalEvaluation.h"
#include "stp/invariants/game_states/NormalOrFreeKickUsGameStateEvaluation.h"
#include "include/roboteam_ai/stp/roles/active/Attacker.h"
#include "include/roboteam_ai/stp/roles/passive/Defender.h"
#include "include/roboteam_ai/stp/roles/passive/Formation.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

Attack::Attack() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalOrFreeKickUsGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalOrFreeKickUsGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToUsInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::Attacker>(role::Attacker("attacker")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_3")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_4")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_1")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_2")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_3"))};
}

uint8_t Attack::score(world::World *world) noexcept {
    if (world->getWorld()->getBall().value()->getPos().dist(field.getTheirGoalCenter()) < field.getFieldLength() / 2) {
        return 150;
    } else
        return 60;
}

Dealer::FlagMap Attack::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag attackerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"attacker", {attackerFlag}});
    flagMap.insert({"offender_1", {closeToTheirGoalFlag}});
    flagMap.insert({"offender_2", {closeToTheirGoalFlag}});
    flagMap.insert({"midfielder_1", {notImportant}});
    flagMap.insert({"midfielder_2", {notImportant}});
    flagMap.insert({"midfielder_3", {notImportant}});
    flagMap.insert({"midfielder_4", {notImportant}});
    flagMap.insert({"defender_1", {notImportant}});
    flagMap.insert({"defender_2", {notImportant}});
    flagMap.insert({"defender_3", {closeToOurGoalFlag}});

    return flagMap;
}

void Attack::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    // Attacker
    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);
    stpInfos["attacker"].setPositionToShootAt(goalTarget);
    stpInfos["attacker"].setShotType(ShotType::MAX);

    // Offenders
    stpInfos["offender_1"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, field.getFieldWidth() / 4));
    stpInfos["offender_2"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, -field.getFieldWidth() / 4));

    // Midfielders
    stpInfos["midfielder_1"].setPositionToMoveTo(Vector2(0.0, field.getFieldWidth() / 4));
    stpInfos["midfielder_2"].setPositionToMoveTo(Vector2(0.0, -field.getFieldWidth() / 4));
    stpInfos["midfielder_3"].setPositionToMoveTo(Vector2(field.getFieldLength() / 8, 0.0));
    stpInfos["midfielder_4"].setPositionToMoveTo(Vector2(-field.getFieldLength() / 8, 0.0));

    // Defenders
    stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_1"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::them));
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_2"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurTopGoalSide(), world::them));
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_3"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_3"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurBottomGoalSide(), world::them));
    stpInfos["defender_3"].setBlockDistance(BlockDistance::HALFWAY);
}

bool Attack::shouldRoleSkipEndTactic() { return false; }

const char *Attack::getName() { return "Attack"; }

}  // namespace rtt::ai::stp::play
