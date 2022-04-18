//
// Created by timovdk on 3/30/20.
//

#include "stp/plays/referee_specific/AggressiveStopFormation.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/passive/BallAvoider.h"

namespace rtt::ai::stp::play {
AggressiveStopFormation::AggressiveStopFormation() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::StopGameState);
    startPlayEvaluation.emplace_back(eval::BallOnTheirSide);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::StopGameState);

    /// Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("defender_0")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("defender_1")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("defender_2")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("mid_field_0")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("mid_field_1")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("mid_field_2")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("offender_0")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("offender_1")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("offender_2")),
                                                                                 std::make_unique<role::BallAvoider>(role::BallAvoider("offender_3"))};
}

uint8_t AggressiveStopFormation::score(const rtt::world::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::BallOnTheirSide, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

void AggressiveStopFormation::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter() + Vector2{0.5, 0.0});
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["defender_0"].setPositionToMoveTo(Vector2{-length / 3.5, 0.0});
    stpInfos["defender_1"].setPositionToMoveTo(Vector2{-length / 3.5, width / 6});
    stpInfos["defender_2"].setPositionToMoveTo(Vector2{-length / 3.5, -width / 6});

    stpInfos["mid_field_0"].setPositionToMoveTo(Vector2{-length / 8, 0.0});
    stpInfos["mid_field_1"].setPositionToMoveTo(Vector2{-length / 9, -width / 4});
    stpInfos["mid_field_2"].setPositionToMoveTo(Vector2{-length / 9, width / 4});

    stpInfos["offender_0"].setPositionToMoveTo(Vector2{length / 8, width / 8});
    stpInfos["offender_1"].setPositionToMoveTo(Vector2{length / 8, width / 4});
    stpInfos["offender_2"].setPositionToMoveTo(Vector2{length / 8, -width / 4});
    stpInfos["offender_3"].setPositionToMoveTo(Vector2{length / 12, -width / 10});  // This robot is put here because BallAvoider doesnt work correctly for KickOffUs
}

Dealer::FlagMap AggressiveStopFormation::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;  // DONT TOUCH.

    /// Creation flagMap. Linking roles to role-priority and the above created flags, can also force ID {roleName, {priority, flags, forceID}}
    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"mid_field_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"mid_field_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"mid_field_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"offender_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"offender_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"offender_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"offender_3", {DealerFlagPriority::REQUIRED, {}}});

    return flagMap;  // DONT TOUCH.
}

const char* AggressiveStopFormation::getName() { return "Aggressive Stop Formation"; }
}  // namespace rtt::ai::stp::play