//
// Created by Tijmen on 22-04-22.
//

#include "stp/plays/referee_specific/FreeKickUsPass.h"

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/FreeKickTaker.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

FreeKickUsPass::FreeKickUsPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::FreeKickUsGameState);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalOrFreeKickUsGameState);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                       std::make_unique<role::FreeKickTaker>(role::FreeKickTaker("free_kick_taker")),
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

uint8_t FreeKickUsPass::score(const rtt::world::Field& field) noexcept {
    passInfo = stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field);

    if (passInfo.passLocation == Vector2()) return 0;  // In case no pass is found

    return stp::computations::PassComputations::scorePass(passInfo, world, field);
}

Dealer::FlagMap FreeKickUsPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}, passInfo.keeperId}});
    flagMap.insert({"free_kick_taker", {DealerFlagPriority::REQUIRED, {}, passInfo.passerId}});
    flagMap.insert({"receiver", {DealerFlagPriority::HIGH_PRIORITY, {}, passInfo.receiverId}});
    flagMap.insert({"defender_left", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_mid", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_right", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"midfielder_left", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"midfielder_mid", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"midfielder_right", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_left", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"attacker_right", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void FreeKickUsPass::calculateInfoForRoles() noexcept {
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
        stpInfos["free_kick_taker"].setPositionToShootAt(passInfo.passLocation);
        stpInfos["free_kick_taker"].setShotType(ShotType::PASS);
        stpInfos["free_kick_taker"].setKickOrChip(KickOrChip::KICK);
    } else {
        // Receiver goes to the passLocation projected on the trajectory of the ball
        auto ball = world->getWorld()->getBall()->get();
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.getFieldLength()));
        auto receiverLocation = FieldComputations::projectPointToValidPositionOnLine(field, passInfo.passLocation, ballTrajectory.start, ballTrajectory.end);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);
        if (ball->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT) stpInfos["receiver"].setPidType(PIDType::INTERCEPT);

        // free_kick_taker now goes to a front grid, where the receiver is not
        if (receiverLocation.y > field.getFrontLeftGrid().getOffSetY()) {  // Receiver is going to left of the field
            stpInfos["free_kick_taker"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleRightGrid(), gen::BlockingPosition, field, world));
        } else if (receiverLocation.y < field.getMiddleMidGrid().getOffSetY()) {  // Receiver is going to right of the field
            stpInfos["free_kick_taker"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::BlockingPosition, field, world));
        } else {  // Receiver is going to middle of the field- free_kick_taker will go to the closest grid on the side of the field
            auto targetGrid = stpInfos["free_kick_taker"].getRobot()->get()->getPos().y < 0 ? field.getMiddleRightGrid() : field.getMiddleLeftGrid();
            stpInfos["free_kick_taker"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, targetGrid, gen::BlockingPosition, field, world));
        }
    }
}

void FreeKickUsPass::calculateInfoForDefenders() noexcept {
    stpInfos["defender_left"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_left"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_mid"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_mid"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_right"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_right"].setBlockDistance(BlockDistance::CLOSE);
}

void FreeKickUsPass::calculateInfoForMidfielders() noexcept {
    stpInfos["midfielder_left"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::OffensivePosition, field, world));
    stpInfos["midfielder_mid"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::OffensivePosition, field, world));
    stpInfos["midfielder_right"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleRightGrid(), gen::OffensivePosition, field, world));
}

void FreeKickUsPass::calculateInfoForAttackers() noexcept {
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

bool FreeKickUsPass::ballKicked() {
    // TODO: create better way of checking when ball has been kicked
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "free_kick_taker" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

bool FreeKickUsPass::shouldEndPlay() noexcept {
    // True if receiver has ball
    if (stpInfos["receiver"].getRobot() && stpInfos["receiver"].getRobot().value()->hasBall()) return true;

    // True if the free_kick_taker has shot the ball, but it is now almost stationary (pass was too soft, was reflected, etc.)
    if (ballKicked() && world->getWorld()->getBall()->get()->velocity.length() < control_constants::BALL_STILL_VEL) return true;

    // True if a different pass has a higher score than the current pass (by some margin)- only if the passer is not already close to the ball (since we don't want to adjust our
    // target when we're in the process of shooting
    return !ballKicked() && stpInfos["receiver"].getRobot() && stpInfos["receiver"].getRobot()->get()->getDistanceToBall() > 0.25 &&
           stp::computations::PassComputations::calculatePass(gen::AttackingPass, world, field).passScore >
               1.05 * stp::PositionScoring::scorePosition(passInfo.passLocation, gen::AttackingPass, field, world).score;
}

const char* FreeKickUsPass::getName() { return "Free Kick Us Pass"; }
}  // namespace rtt::ai::stp::play