//
// Created by jessevw on 17.03.20.
/// TODO-Max change to fowardPass
//

#include "stp/plays/offensive/AttackingPass.h"

#include <roboteam_utils/LineSegment.h>

#include "stp/computations/PassComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/Formation.h"
#include "world/views/RobotView.hpp"

namespace rtt::ai::stp::play {
AttackingPass::AttackingPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")), std::make_unique<role::Passer>(role::Passer("passer")),
        std::make_unique<role::PassReceiver>(role::PassReceiver("receiver")), std::make_unique<role::Formation>(role::Formation("midfielder"))};
}

uint8_t AttackingPass::score(PlayEvaluator& playEvaluator) noexcept {
    auto world = playEvaluator.getWorld();
    auto field = world->getField().value();
    passLocation = calculatePassLocation(world).position;
    if (passLocation == Vector2()) return 0;  // In case no pass is found

    calculateInfoForScoredRoles(world);

    // Score of play is the goalshotscore, adjusted based on the LoS and openness scores. The worse the LoS/Openness, the more the score is reduced
    auto goalShotScore = static_cast<int>(PositionScoring::scorePosition(passLocation.value(), gen::GoalShot, field, world).score);
    auto lineOfSightScore = static_cast<int>(PositionScoring::scorePosition(passLocation.value(), gen::LineOfSight, field, world).score);
    auto openScore = static_cast<int>(PositionScoring::scorePosition(passLocation.value(), gen::Open, field, world).score);
    return std::clamp(static_cast<int>(goalShotScore * (lineOfSightScore / 255.0) * (openScore / 255.0)), 0, 255);
}

Dealer::FlagMap AttackingPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag passerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag receiverFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"passer", {DealerFlagPriority::REQUIRED, {passerFlag}}});
    flagMap.insert({"receiver", {DealerFlagPriority::HIGH_PRIORITY, {receiverFlag}}});
    flagMap.insert({"midfielder", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void AttackingPass::calculateInfoForRoles() noexcept {
    calculateInfoForScoredRoles(world);

    /// Keeper
    stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    // TODO: set good pass position
    stpInfos["keeper"].setPositionToShootAt(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::us).value()->getPos());
    stpInfos["keeper"].setKickOrChip(KickOrChip::CHIP);

    /// Midfielder
    stpInfos["midfielder"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getMiddleMidGrid(), gen::SafePosition, field, world));
}

void AttackingPass::calculateInfoForScoredRoles(world::World* world) noexcept {
    if (!passLocation) passLocation = calculatePassLocation(world).position;
    auto ball = world->getWorld()->getBall()->get();
    auto field = world->getField().value();
    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(passLocation.value());
        stpInfos["passer"].setPositionToShootAt(passLocation.value());
        stpInfos["passer"].setShotType(ShotType::PASS);
    } else {
        // Receiver goes to the passLocation projected on the trajectory of the ball
        auto ballTrajectory = LineSegment(ball->getPos(), ball->getPos() + ball->getFilteredVelocity().stretchToLength(field.getFieldLength()));
        auto receiverLocation = ballTrajectory.project(passLocation.value());
        receiverLocation =
            PositionComputations::ProjectPositionIntoFieldOnLine(field, receiverLocation, ballTrajectory.start, ballTrajectory.end, -2 * control_constants::ROBOT_RADIUS);
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);

        // Passer now goes to a front grid, where the receiver is not
        if (receiverLocation.y > field.getFrontLeftGrid().getOffSetY()) {  // Receiver is going to left of the field
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontRightGrid(), gen::OffensivePosition, field, world));
        } else if (receiverLocation.y < field.getMiddleMidGrid().getOffSetY()) {  // Receiver is going to right of the field
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.getFrontLeftGrid(), gen::OffensivePosition, field, world));
        } else {  // Receiver is going to middle of the field- passer will go to the closest grid on the side of the field
            auto targetGrid = stpInfos["passer"].getRobot()->get()->getPos().y < 0 ? field.getFrontRightGrid() : field.getFrontLeftGrid();
            stpInfos["passer"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, targetGrid, gen::OffensivePosition, field, world));
        }
    }
}

bool AttackingPass::ballKicked() {
    // TODO: create better way of checking when ball has been kicked
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role>& role) {
        return role != nullptr && role->getName() == "passer" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}
const char* AttackingPass::getName() { return "AttackingPass"; }

bool AttackingPass::shouldEndPlay() noexcept {
    if (stpInfos["receiver"].getRobot() && stpInfos["passer"].getRobot()) {
        // True if receiver has ball
        if (stpInfos["receiver"].getRobot()->hasBall()) return true;

        // True if the passer has shot the ball, but it is now stationary (pass was too soft, was reflected, etc.)
        return ballKicked() && stpInfos["passer"].getRobot()->get()->getDistanceToBall() >= control_constants::HAS_BALL_DISTANCE_ERROR_MARGIN * 1.5 &&
               world->getWorld()->getBall()->get()->getVelocity().length() < control_constants::BALL_STILL_VEL;
    }
    return false;
}

gen::ScoredPosition AttackingPass::calculatePassLocation(world::World* world) {
    auto ball = world->getWorld()->getBall()->get();
    auto field = world->getField().value();
    // TODO: filter which robots can receive in a more consistent way
    auto possibleReceivers = world->getWorld()->getUs();

    // If we only have 2 or less robots, return zero pos and score
    if (possibleReceivers.size() <= 2) return {Vector2(0, 0), 0};

    auto passerRobot = world->getWorld()->getRobotClosestToBall(world::us)->get();
    // Remove passer and keeper from possible receivers TODO: Do this in a less hacky way
    std::erase_if(possibleReceivers, [&](const rtt::ai::stp::world::view::RobotView& receiver) { return receiver->getId() == passerRobot->getId(); });
    std::erase_if(possibleReceivers, [&](const rtt::ai::stp::world::view::RobotView& receiver) {
        return receiver->getId() == world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::us)->get()->getId();
    });

    std::vector<Vector2> possibleReceiverLocations;
    possibleReceiverLocations.reserve(possibleReceivers.size());
    for (auto& receiver : possibleReceivers) {
        possibleReceiverLocations.emplace_back(receiver->getPos());
    }
    return computations::PassComputations::calculatePassLocation(ball->getPos(), possibleReceiverLocations, passerRobot->getPos(), world, field);
}

void AttackingPass::storePlayInfo(gen::PlayInfos& info) noexcept {
    passLocation = std::nullopt;  // Reset pass location to ensure it is recalculated next time this play is executed
}
}  // namespace rtt::ai::stp::play
