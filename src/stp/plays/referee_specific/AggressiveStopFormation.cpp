//
// Created by timovdk on 3/30/20.
//

#include "include/roboteam_ai/stp/plays/referee_specific/AggressiveStopFormation.h"
#include "include/roboteam_ai/stp/roles/passive/BallAvoider.h"
#include "include/roboteam_ai/stp/roles/Keeper.h"

namespace rtt::ai::stp::play {
    AggressiveStopFormation::AggressiveStopFormation() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear(); // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::StopGameState);
    startPlayEvaluation.emplace_back(eval::BallOnTheirSide);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear(); // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::StopGameState);

    /// Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
            std::make_unique<role::Keeper>(role::Keeper("keeper")),
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
    initRoles(); // DONT TOUCH.
}

    uint8_t AggressiveStopFormation::score(PlayEvaluator& playEvaluator) noexcept {
        /// List of all factors that combined results in an evaluation how good the play is.
        scoring = {{playEvaluator.getGlobalEvaluation(eval::BallOnTheirSide),1.0}};
        return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
    }

void AggressiveStopFormation::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter() + Vector2{0.5, 0.0});
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["defender_0"].setPositionToMoveTo(Vector2{-length / 5, 0.0});
    stpInfos["defender_1"].setPositionToMoveTo(Vector2{-length / 5, width / 6});
    stpInfos["defender_2"].setPositionToMoveTo(Vector2{-length / 5, -width / 6});

    stpInfos["mid_field_0"].setPositionToMoveTo(Vector2{-length / 8, 0.0});
    stpInfos["mid_field_1"].setPositionToMoveTo(Vector2{-length / 9, -width / 4});
    stpInfos["mid_field_2"].setPositionToMoveTo(Vector2{-length / 9, width / 4});

    stpInfos["offender_0"].setPositionToMoveTo(Vector2{length / 4, 0.0});
    stpInfos["offender_1"].setPositionToMoveTo(Vector2{length / 4, width / 4});
    stpInfos["offender_2"].setPositionToMoveTo(Vector2{length / 4, -width / 4});
    stpInfos["offender_3"].setPositionToMoveTo(Vector2{length / 3, 0.0});

    /// Using the positions below is better, but right now the pos::getPosition() is so slow it crashes the interface
//    stpInfos["defender_0"].setPositionToMoveTo(pos::getWallPosition(0, 3, field, world));
//    stpInfos["defender_1"].setPositionToMoveTo(pos::getWallPosition(1, 3, field, world));
//    stpInfos["defender_2"].setPositionToMoveTo(pos::getWallPosition(2, 3, field, world));
//
//    stpInfos["mid_field_0"].setPositionToMoveTo(pos::getPosition(stpInfos["mid_field_0"].getPositionToMoveTo(),gen::gridLeftMid, gen::BlockingPosition, field, world));
//    stpInfos["mid_field_1"].setPositionToMoveTo(pos::getPosition(stpInfos["mid_field_1"].getPositionToMoveTo(),gen::gridMidFieldTop, gen::OffensivePosition, field, world));
//    stpInfos["mid_field_2"].setPositionToMoveTo(pos::getPosition(stpInfos["mid_field_2"].getPositionToMoveTo(),gen::gridMidFieldBot, gen::BlockingPosition, field, world));
//
//    stpInfos["offender_0"].setPositionToMoveTo(pos::getPosition(stpInfos["offender_0"].getPositionToMoveTo(),gen::gridRightMid, gen::OffensivePosition, field, world));
//    stpInfos["offender_1"].setPositionToMoveTo(pos::getPosition(stpInfos["offender_1"].getPositionToMoveTo(),gen::gridRightMid, gen::OffensivePosition, field, world));
//    stpInfos["offender_2"].setPositionToMoveTo(pos::getPosition(stpInfos["offender_2"].getPositionToMoveTo(),gen::gridRightTop, gen::GoalShootPosition, field, world));
//    stpInfos["offender_3"].setPositionToMoveTo(pos::getPosition(stpInfos["offender_3"].getPositionToMoveTo(),gen::gridRightBot, gen::GoalShootPosition, field, world));
}

    Dealer::FlagMap AggressiveStopFormation::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap; // DONT TOUCH.

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
        flagMap.insert({"offender_3", {DealerFlagPriority::LOW_PRIORITY, {}}});

        return flagMap; // DONT TOUCH.
    }

    const char* AggressiveStopFormation::getName() { return "Aggressive Stop Formation"; }
}  // namespace rtt::ai::stp::play