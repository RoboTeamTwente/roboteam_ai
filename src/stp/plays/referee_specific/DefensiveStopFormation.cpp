//
// Created by timo on 3/27/20.
//

#include "stp/plays/referee_specific/DefensiveStopFormation.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallAvoider.h"

namespace rtt::ai::stp::play {

DefensiveStopFormation::DefensiveStopFormation() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::StopGameState);
    startPlayEvaluation.emplace_back(eval::BallOnOurSide);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::StopGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("defender_0")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("defender_1")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("defender_2")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("defender_3")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("mid_field_0")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("mid_field_1")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("mid_field_2")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("offender_0")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("offender_1")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("offender_2"))};
}

uint8_t DefensiveStopFormation::score(const rtt::world::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::BallOnOurSide, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

void DefensiveStopFormation::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter() + Vector2{0.5, 0.0});
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["defender_0"].setPositionToMoveTo(Vector2(-length / 4, width / 8));
    stpInfos["defender_1"].setPositionToMoveTo(Vector2(-length / 4, -width / 8));
    stpInfos["defender_2"].setPositionToMoveTo(Vector2(-length / 4.5, width / 3));
    stpInfos["defender_3"].setPositionToMoveTo(Vector2(-length / 4.5, -width / 3));

    stpInfos["mid_field_0"].setPositionToMoveTo(Vector2{-length / 8, 0.0});
    stpInfos["mid_field_1"].setPositionToMoveTo(Vector2{-length / 9, -width / 4});
    stpInfos["mid_field_2"].setPositionToMoveTo(Vector2{-length / 9, width / 4});

    stpInfos["offender_0"].setPositionToMoveTo(Vector2{length / 12, width / 10});  // This robot is put here because BallAvoider doesnt work correctly for KickOffUs
    stpInfos["offender_1"].setPositionToMoveTo(Vector2{length / 8, width / 4});
    stpInfos["offender_2"].setPositionToMoveTo(Vector2{length / 8, -width / 4});
}

Dealer::FlagMap DefensiveStopFormation::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    /// Creation flagMap. Linking roles to role-priority and the above created flags, can also force ID {roleName, {priority, flags, forceID}}
    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"mid_field_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"mid_field_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"mid_field_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"offender_0", {DealerFlagPriority::REQUIRED, {}}});
    flagMap.insert({"offender_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"offender_2", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

const char* DefensiveStopFormation::getName() { return "Defensive Stop Formation"; }
}  // namespace rtt::ai::stp::play