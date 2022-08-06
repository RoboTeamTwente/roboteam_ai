//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/BallPlacementUs.h"

#include "stp/roles/active/BallPlacer.h"
#include "stp/roles/passive/BallAvoider.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::play {

BallPlacementUs::BallPlacementUs() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::BallPlacementUsGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::BallPlacementUsGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::BallPlacer>(role::BallPlacer("ball_placer")),      std::make_unique<role::BallAvoider>(role::BallAvoider("keeper")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_0")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_1")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_2")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_3")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_4")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_5")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_6")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_7")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_8"))};
}

uint8_t BallPlacementUs::score(const rtt::world::Field& field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::BallPlacementUsGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

void BallPlacementUs::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    auto ballTarget = rtt::ai::GameStateManager::getRefereeDesignatedPosition();

    // Adjust placement position to be one robot radius away in the distance of movement
    if (stpInfos["ball_placer"].getRobot())
        ballTarget -= (world->getWorld()->get()->getBall()->get()->position - stpInfos["ball_placer"].getRobot()->get()->getPos()).stretchToLength(control_constants::ROBOT_RADIUS);

    stpInfos["ball_placer"].setPositionToShootAt(ballTarget);
    stpInfos["ball_placer"].setPositionToMoveTo(ballTarget);
    stpInfos["ball_placer"].setShouldAvoidDefenseArea(false);
    stpInfos["ball_placer"].setShouldAvoidOutOfField(false);

    if (stpInfos["ball_placer"].getRobot() && stpInfos["ball_placer"].getRobot()->get()->getDistanceToBall() < 1.0) stpInfos["ball_placer"].setMaxRobotVelocity(0.75);
    if (stpInfos["ball_placer"].getRobot() && stpInfos["ball_placer"].getRobot()->get()->getDistanceToBall() < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        stpInfos["ball_placer"].setDribblerSpeed(100);
    }

    /// TODO: figure out what the rest of the robots should do during our ball placement
}

Dealer::FlagMap BallPlacementUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag ballPlacementPriority(DealerFlagTitle::CAN_DETECT_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag ballPlacementPreference(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"ball_placer", {DealerFlagPriority::REQUIRED, {ballPlacementPriority, ballPlacementPreference}}});
    flagMap.insert({"ball_avoider_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

const char* BallPlacementUs::getName() { return "Ball Placement Us"; }
}  // namespace rtt::ai::stp::play
