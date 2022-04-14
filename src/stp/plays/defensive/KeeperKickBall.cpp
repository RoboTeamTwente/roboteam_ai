//
// Created by Alexander on 29/01/2022.
//

#include "stp/plays/defensive/KeeperKickBall.h"

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

KeeperKickBall::KeeperKickBall() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::BallInOurDefenseAreaAndStill);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::TheyDoNotHaveBall);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Passer>(role::Passer("keeper")),
        std::make_unique<role::PassReceiver>(role::PassReceiver("receiver")),
        std::make_unique<role::Formation>(role::Formation("midfielder")),
        std::make_unique<role::Formation>(role::Formation("defender")),
    };
}

uint8_t KeeperKickBall::score(const rtt::world::Field& field) noexcept {
    // Calculate passInfo to be used during the play
    passInfo = stp::computations::PassComputations::calculatePass(gen::SafePass, world, field);

    // If this play is valid, the ball is in the defense area and still, and we always want to execute this play
    return control_constants::FUZZY_TRUE;
}

Dealer::FlagMap KeeperKickBall::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}, passInfo.keeperId}});
    flagMap.insert({"receiver", {DealerFlagPriority::HIGH_PRIORITY, {}, passInfo.receiverId}});
    flagMap.insert({"midfielder", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void KeeperKickBall::calculateInfoForRoles() noexcept {
    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(passInfo.passLocation);
        stpInfos["keeper"].setPositionToShootAt(passInfo.passLocation);
        stpInfos["keeper"].setShotType(ShotType::PASS);
    } else {
        auto ball = world->getWorld()->getBall().value();
        // Receiver goes to the passLocation projected on the trajectory of the ball
        auto ballTrajectory = LineSegment(ball->getPos(), ball->getPos() + ball->getFilteredVelocity().stretchToLength(field.getFieldLength()));
        auto receiverLocation = ballTrajectory.project(passInfo.passLocation);
        receiverLocation =
            PositionComputations::ProjectPositionIntoFieldOnLine(field, receiverLocation, ballTrajectory.start, ballTrajectory.end, -2 * control_constants::ROBOT_RADIUS);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);

        // Keeper goes back to his goal
        // TODO: go back to KeeperBlockBall tactic
        stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter() + Vector2(0.2, 0));
    }
    // Passer now goes to a front grid, where the receiver is not
    if (passInfo.passLocation.y > field.getFrontLeftGrid().getOffSetY()) {  // Receiver is going to left of the field
        stpInfos["defender"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getBackRightGrid(), gen::SafePosition, field, world));
        stpInfos["midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::OffensivePosition, field, world));
    } else if (passInfo.passLocation.y < field.getMiddleMidGrid().getOffSetY()) {  // Receiver is going to right of the field
        stpInfos["defender"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getBackLeftGrid(), gen::SafePosition, field, world));
        stpInfos["midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::OffensivePosition, field, world));
    } else {  // Receiver is going to middle of the field- passer will go to the closest grid on the side of the field
        stpInfos["defender"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getBackRightGrid(), gen::SafePosition, field, world));
        stpInfos["midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::OffensivePosition, field, world));
    }
}

bool KeeperKickBall::ballKicked() {
    // TODO: create better way of checking when ball has been kicked
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "keeper" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

bool KeeperKickBall::shouldEndPlay() noexcept {
    if (stpInfos["receiver"].getRobot() && stpInfos["keeper"].getRobot()) {
        // True if receiver has ball
        if (stpInfos["receiver"].getRobot()->hasBall()) return true;

        // True if the passer has shot the ball, but it is now almost stationary (pass was too soft, was reflected, etc.)
        if (ballKicked() && stpInfos["keeper"].getRobot()->get()->getDistanceToBall() >= control_constants::HAS_BALL_DISTANCE_ERROR_MARGIN * 1.5 &&
            world->getWorld()->getBall()->get()->getVelocity().length() < control_constants::BALL_STILL_VEL)
            return true;
    }
    // True if a different pass has a higher score than the current pass (by some margin)
    return !ballKicked() && stp::computations::PassComputations::calculatePass(gen::SafePass, world, field).passScore >
                                1.05 * stp::PositionScoring::scorePosition(passInfo.passLocation, gen::SafePass, field, world).score;
}
const char* KeeperKickBall::getName() { return "Keeper Kick Ball"; }

}  // namespace rtt::ai::stp::play
