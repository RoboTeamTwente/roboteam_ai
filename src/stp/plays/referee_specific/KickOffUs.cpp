//
// Created by timovdk on 5/1/20.
//

#include "stp/plays/referee_specific/KickOffUs.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/active/FreeKickTaker.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/passive/Halt.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::play {

KickOffUs::KickOffUs() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::KickOffUsGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::TheyDoNotHaveBall);
    keepPlayEvaluation.emplace_back(eval::KickOffUsOrNormalGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>("keeper"),
                                                                                 std::make_unique<role::FreeKickTaker>("kick_off_taker"),
                                                                                 std::make_unique<role::PassReceiver>("receiver"),
                                                                                 std::make_unique<role::Halt>("halt_0"),
                                                                                 std::make_unique<role::Halt>("halt_1"),
                                                                                 std::make_unique<role::Halt>("halt_2"),
                                                                                 std::make_unique<role::Halt>("halt_3"),
                                                                                 std::make_unique<role::Halt>("halt_4"),
                                                                                 std::make_unique<role::Halt>("halt_5"),
                                                                                 std::make_unique<role::Halt>("halt_6"),
                                                                                 std::make_unique<role::Halt>("halt_7")};
}

uint8_t KickOffUs::score(const rtt::world::Field &field) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{PlayEvaluator::getGlobalEvaluation(eval::KickOffUsGameState, world), 1.0}};
    return (lastScore = PlayEvaluator::calculateScore(scoring)).value();  // DONT TOUCH.
}

void KickOffUs::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter() + Vector2(control_constants::DISTANCE_FROM_GOAL_CLOSE, 0));
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    // Kicker
    // TODO: set good position to shoot at (compute pass location)- possibly do this in the kick_off_taker role
    Vector2 passLocation = Vector2(-1.0, 1.0);
    stpInfos["kick_off_taker"].setPositionToShootAt(passLocation);
    stpInfos["kick_off_taker"].setShotType(ShotType::PASS);
    stpInfos["kick_off_taker"].setKickOrChip(KickOrChip::KICK);
    // TODO: set good position to move to after pass
    stpInfos["kick_off_taker"].setPositionToMoveTo(Vector2(0, 0));

    // Receiver
    // TODO: set receiving position based on pass computation
    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(passLocation);
    } else {
        auto ball = world->getWorld()->getBall()->get();
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.getFieldLength()));
        auto receiverLocation = FieldComputations::projectPointToValidPositionOnLine(field, passLocation, ballTrajectory.start, ballTrajectory.end);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);
        if (ball->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT) stpInfos["receiver"].setPidType(PIDType::INTERCEPT);
    }
}

Dealer::FlagMap KickOffUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);

    Dealer::DealerFlag kickerFirstPriority(DealerFlagTitle::CAN_KICK_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag kickerSecondPriority(DealerFlagTitle::CAN_DETECT_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag kickerThirdPriority(DealerFlagTitle::CLOSEST_TO_BALL, DealerFlagPriority::MEDIUM_PRIORITY);

    Dealer::DealerFlag receiverFlag(DealerFlagTitle::CAN_DETECT_BALL, DealerFlagPriority::HIGH_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"kick_off_taker", {DealerFlagPriority::REQUIRED, {kickerFirstPriority, kickerSecondPriority, kickerThirdPriority}}});
    flagMap.insert({"receiver", {DealerFlagPriority::HIGH_PRIORITY, {receiverFlag}}});
    flagMap.insert({"halt_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

bool KickOffUs::shouldEndPlay() noexcept {
    if (stpInfos["receiver"].getRobot() && stpInfos["receiver"].getRobot().value()->hasBall()) return true;

    if (ballKicked() && world->getWorld()->getBall()->get()->velocity.length() < control_constants::BALL_IS_MOVING_SLOW_LIMIT * 0.5)
        return true;

    return false;
}

bool KickOffUs::ballKicked() {
    // TODO: create better way of checking when ball has been kicked
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role> &role) {
        return role != nullptr && role->getName() == "kick_off_taker" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

const char *KickOffUs::getName() { return "Kick Off Us"; }
}  // namespace rtt::ai::stp::play
