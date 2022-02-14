//
// Created by Alexander on 29/01/2022.
//

#include "stp/plays/defensive/KeeperKickBall.h"

#include "stp/computations/PassComputations.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/Formation.h"
#include "utilities/GameStateManager.hpp"

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

uint8_t KeeperKickBall::score(PlayEvaluator& playEvaluator) noexcept {
    auto world = playEvaluator.getWorld();
    auto field = world->getField().value();
    return control_constants::FUZZY_TRUE;
}

Dealer::FlagMap KeeperKickBall::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"receiver", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"midfielder", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void KeeperKickBall::calculateInfoForRoles() noexcept {
    if (!passLocation) passLocation = calculatePassLocation(world);
    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(passLocation);
        stpInfos["keeper"].setPositionToShootAt(passLocation);
        stpInfos["keeper"].setShotType(ShotType::PASS);
    } else {
        auto ball = world->getWorld()->getBall().value();
        // Receiver goes to the passLocation projected on the trajectory of the ball
        auto ballTrajectory = LineSegment(ball->getPos(), ball->getPos() + ball->getFilteredVelocity().stretchToLength(field.getFieldLength()));
        auto receiverLocation = ballTrajectory.project(passLocation.value());
        receiverLocation =
            PositionComputations::ProjectPositionIntoFieldOnLine(field, receiverLocation, ballTrajectory.start, ballTrajectory.end, -2 * control_constants::ROBOT_RADIUS);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);

        // Keeper goes back to his goal
        // TODO: go back to KeeperBlockBall tactic
        stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter() + Vector2(0.2, 0));
    }
    // Passer now goes to a front grid, where the receiver is not
    if (passLocation.value().y > field.getFrontLeftGrid().getOffSetY()) {  // Receiver is going to left of the field
        stpInfos["defender"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getBackRightGrid(), gen::SafePosition, field, world));
        stpInfos["midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::SafePosition, field, world));
    } else if (passLocation.value().y < field.getMiddleMidGrid().getOffSetY()) {  // Receiver is going to right of the field
        stpInfos["defender"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getBackLeftGrid(), gen::SafePosition, field, world));
        stpInfos["midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::SafePosition, field, world));
    } else {  // Receiver is going to middle of the field- passer will go to the closest grid on the side of the field
        stpInfos["defender"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getBackRightGrid(), gen::OffensivePosition, field, world));
        stpInfos["midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleLeftGrid(), gen::OffensivePosition, field, world));
    }
}

Vector2 KeeperKickBall::calculatePassLocation(world::World* world) {
    auto ball = world->getWorld()->getBall()->get();
    auto field = world->getField().value();
    auto possibleReceivers = world->getWorld()->getUs();

    // If we only have 2 or less robots, return zero pos and score
    if (possibleReceivers.size() <= 1) return Vector2(0.0, 0.0);

    auto keeperIt = std::find_if(possibleReceivers.begin(), possibleReceivers.end(),
                                 [&](const rtt::ai::stp::world::view::RobotView& receiver) { return receiver->getId() == GameStateManager::getCurrentGameState().keeperId; });
    Vector2 keeperPos = ball->getPos();  // Default keeperPos to ballPos in case we can't find the keeper based on id (i.e. there is no keeper yet)
    if (keeperIt != possibleReceivers.end()) {
        keeperPos = (*keeperIt)->getPos();
        // Remove keeper from possible receivers
        possibleReceivers.erase(keeperIt);
    }

    std::vector<Vector2> possibleReceiverLocations;
    possibleReceiverLocations.reserve(possibleReceivers.size());
    for (auto& receiver : possibleReceivers) {
        possibleReceiverLocations.emplace_back(receiver->getPos());
    }
    return computations::PassComputations::calculatePassLocation(ball->getPos(), possibleReceiverLocations, keeperPos, gen::SafePass, world, field).position;
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

        // True if the passer has shot the ball, but it is now stationary (pass was too soft, was reflected, etc.)
        return ballKicked() && stpInfos["keeper"].getRobot()->get()->getDistanceToBall() >= control_constants::HAS_BALL_DISTANCE_ERROR_MARGIN * 1.5 &&
               world->getWorld()->getBall()->get()->getVelocity().length() < control_constants::BALL_STILL_VEL;
    }
    return false;
}
const char* KeeperKickBall::getName() { return "Keeper Kick Ball"; }

}  // namespace rtt::ai::stp::play
