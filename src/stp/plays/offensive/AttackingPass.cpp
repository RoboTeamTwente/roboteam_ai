//
// Created by jessevw on 17.03.20.
/// TODO-Max change to fowardPass
//

#include "stp/plays/offensive/AttackingPass.h"

#include <roboteam_utils/LineSegment.h>

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/constants/ControlConstants.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"
#include "world/views/RobotView.hpp"

namespace rtt::ai::stp::play {
AttackingPass::AttackingPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                       std::make_unique<role::Passer>(role::Passer("passer")),
                                                                                       std::make_unique<role::PassReceiver>(role::PassReceiver("receiver")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("defender_left")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("defender_mid")),
                                                                                       std::make_unique<role::BallDefender>(role::BallDefender("defender_right")),
                                                                                       std::make_unique<role::Formation>(role::Formation("midfielder_left")),
                                                                                       std::make_unique<role::Formation>(role::Formation("midfielder_mid")),
                                                                                       std::make_unique<role::Formation>(role::Formation("midfielder_right")),
                                                                                       std::make_unique<role::Formation>(role::Formation("attacker_left")),
                                                                                       std::make_unique<role::Formation>(role::Formation("attacker_right"))};
}

uint8_t AttackingPass::score(const rtt::world::Field& field) noexcept {
    passInfo = stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field);

    if (passInfo.passLocation == Vector2()) return 0;  // In case no pass is found

    return stp::computations::PassComputations::scorePass(passInfo, world, field);
}

Dealer::FlagMap AttackingPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}, passInfo.keeperId}});
    flagMap.insert({"passer", {DealerFlagPriority::REQUIRED, {}, passInfo.passerId}});
    flagMap.insert({"receiver", {DealerFlagPriority::HIGH_PRIORITY, {}, passInfo.receiverId}});
    flagMap.insert({"defender_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"defender_mid", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_right", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"midfielder_mid", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"midfielder_right", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_left", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"attacker_right", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void AttackingPass::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForMidfielders();
    calculateInfoForAttackers();

    /// Keeper
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    /// Midfielder
    stpInfos["midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::SafePosition, field, world));

    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(passInfo.passLocation);
        stpInfos["passer"].setPositionToShootAt(passInfo.passLocation);
        stpInfos["passer"].setShotType(ShotType::PASS);
        stpInfos["passer"].setKickOrChip(KickOrChip::KICK);
    } else {
        // Receiver goes to the passLocation projected on the trajectory of the ball
        auto ball = world->getWorld()->getBall()->get();
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.getFieldLength()));
        auto receiverLocation = FieldComputations::projectPointToValidPositionOnLine(field, passInfo.passLocation, ballTrajectory.start, ballTrajectory.end);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);
        if (ball->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT) stpInfos["receiver"].setPidType(PIDType::INTERCEPT);

        // Passer now goes to a front grid, where the receiver is not
        if (receiverLocation.y > field.getFrontLeftGrid().getOffSetY()) {  // Receiver is going to left of the field
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleRightGrid(), gen::BlockingPosition, field, world));
        } else if (receiverLocation.y < field.getMiddleMidGrid().getOffSetY()) {  // Receiver is going to right of the field
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::BlockingPosition, field, world));
        } else {  // Receiver is going to middle of the field- passer will go to the closest grid on the side of the field
            auto targetGrid = stpInfos["passer"].getRobot()->get()->getPos().y < 0 ? field.getMiddleRightGrid() : field.getMiddleLeftGrid();
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, targetGrid, gen::BlockingPosition, field, world));
        }
    }
}

void AttackingPass::calculateInfoForDefenders() noexcept {
    stpInfos["defender_left"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_left"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_mid"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_mid"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_right"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_right"].setBlockDistance(BlockDistance::CLOSE);
}

void AttackingPass::calculateInfoForMidfielders() noexcept {
    stpInfos["midfielder_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::OffensivePosition, field, world));
    stpInfos["midfielder_mid"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::OffensivePosition, field, world));
    stpInfos["midfielder_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleRightGrid(), gen::OffensivePosition, field, world));
}

void AttackingPass::calculateInfoForAttackers() noexcept {
    if (passInfo.passLocation.y > field.getFrontLeftGrid().getOffSetY()) {  // Receiver is going to left of the field
        stpInfos["attacker_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontMidGrid(), gen::OffensivePosition, field, world));
        stpInfos["attacker_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
    } else if (passInfo.passLocation.y < field.getMiddleMidGrid().getOffSetY()) {  // Receiver is going to right of the field
        stpInfos["attacker_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
        stpInfos["attacker_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontMidGrid(), gen::OffensivePosition, field, world));
    } else {  // Receiver is going to middle of the field
        stpInfos["attacker_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
        stpInfos["attacker_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
    }
}

bool AttackingPass::ballKicked() {
    // TODO: create better way of checking when ball has been kicked
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "passer" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

bool AttackingPass::shouldEndPlay() noexcept {
    // If the receiver has the ball, the play finished successfully
    if (stpInfos["receiver"].getRobot() && stpInfos["receiver"].getRobot().value()->hasBall()) return true;

    // If the ball is moving too slow after we have kicked it, we should stop the play to get the ball
    if (ballKicked() && world->getWorld()->getBall()->get()->velocity.length() < control_constants::BALL_IS_MOVING_SLOW_LIMIT) return true;

    // If the passer doesn't have the ball yet and there is a better pass available, we should stop the play
    if (!ballKicked() && stpInfos["passer"].getRobot() && !stpInfos["passer"].getRobot().value()->hasBall() &&
        stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field).passScore >
            1.05 * stp::PositionScoring::scorePosition(passInfo.passLocation, gen::AttackingPass, field, world).score)
        return true;

    // TODO: probably should do this via an invariant (BALL_IS_NOT_MOVING_FAST or something similar)
    if (!ballKicked() && world->getWorld()->getBall()->get()->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT)
        return true;

    return false;
}

const char* AttackingPass::getName() { return "AttackingPass"; }
}  // namespace rtt::ai::stp::play
