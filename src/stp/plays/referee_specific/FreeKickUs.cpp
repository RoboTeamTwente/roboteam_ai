//
// Created by Floris Hoek on 22-06-21.
//

#include "stp/computations/GoalComputations.h"
#include "stp/plays/referee_specific/FreeKickUs.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Attacker.h"
#include "stp/roles/passive/BallAvoider.h"

namespace rtt::ai::stp::play {

FreeKickUs::FreeKickUs() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>("keeper"),
                                                                                 std::make_unique<role::Attacker>("attacker"),
                                                                                 std::make_unique<role::BallAvoider>("offender_1"),
                                                                                 std::make_unique<role::BallAvoider>("offender_2"),
                                                                                 std::make_unique<role::BallAvoider>("midfielder_1"),
                                                                                 std::make_unique<role::BallAvoider>("midfielder_2"),
                                                                                 std::make_unique<role::BallAvoider>("midfielder_3"),
                                                                                 std::make_unique<role::BallAvoider>("midfielder_4"),
                                                                                 std::make_unique<role::BallAvoider>("defender_1"),
                                                                                 std::make_unique<role::BallAvoider>("defender_2"),
                                                                                 std::make_unique<role::BallAvoider>("defender_3")};
}

uint8_t FreeKickUs::score(PlayEvaluator& playEvaluator) noexcept {
//    /// List of all factors that combined results in an evaluation how good the play is.
//    scoring = {{playEvaluator.getGlobalEvaluation(eval::FreeKickUsGameState), 1.0}};
//    return (lastScore = playEvaluator.calculateScore(scoring)).value();  // DONT TOUCH.
    return 160;
}

Dealer::FlagMap FreeKickUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::FlagInstruction attackerFlag(Flag::CLOSE_TO_BALL, Priority::REQUIRED);
    Dealer::FlagInstruction closeToTheirGoalFlag(Flag::CLOSE_TO_THEIR_GOAL, Priority::MEDIUM_PRIORITY);
    Dealer::FlagInstruction closeToOurGoalFlag(Flag::CLOSE_TO_OUR_GOAL, Priority::MEDIUM_PRIORITY);
    Dealer::FlagInstruction notImportant(Flag::NOT_IMPORTANT, Priority::LOW_PRIORITY);

    flagMap.insert({"keeper", {Priority::KEEPER, {}}});
    flagMap.insert({"attacker", {Priority::REQUIRED, {attackerFlag}}});
    flagMap.insert({"offender_1", {Priority::HIGH_PRIORITY, {closeToTheirGoalFlag}}});
    flagMap.insert({"offender_2", {Priority::HIGH_PRIORITY, {closeToTheirGoalFlag}}});
    flagMap.insert({"midfielder_1", {Priority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_2", {Priority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_3", {Priority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"midfielder_4", {Priority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"defender_1", {Priority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"defender_2", {Priority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"defender_3", {Priority::MEDIUM_PRIORITY, {closeToOurGoalFlag}}});

    return flagMap;
}

void FreeKickUs::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    // Attacker
    auto goalTarget = computations::GoalComputations::calculateGoalTarget(world, field);

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["attacker"].setPositionToShootAt(goalTarget);
    stpInfos["attacker"].setShotType(ShotType::PASS);

    stpInfos["defender_1"].setPositionToMoveTo(Vector2{-length / 3.5, 0.0});
    stpInfos["defender_2"].setPositionToMoveTo(Vector2{-length / 3.5, width / 6});
    stpInfos["defender_3"].setPositionToMoveTo(Vector2{-length / 3.5, -width / 6});


    stpInfos["mid_field_1"].setPositionToMoveTo(Vector2{-length / 8, 0.5});
    stpInfos["mid_field_2"].setPositionToMoveTo(Vector2{-length / 9, -width / 4});
    stpInfos["mid_field_3"].setPositionToMoveTo(Vector2{-length / 9, width / 4});
    stpInfos["mid_field_4"].setPositionToMoveTo(Vector2{-length / 8, -0.5});

    stpInfos["offender_1"].setPositionToMoveTo(Vector2{length / 8, -width / 4});
    stpInfos["offender_2"].setPositionToMoveTo(Vector2{length / 8, width / 4});
}

const char* FreeKickUs::getName() { return "Free Kick Us"; }

}  // namespace rtt::ai::stp::play
