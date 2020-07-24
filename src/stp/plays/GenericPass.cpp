//
// Created by timovdk on 5/20/20.
//

#include "stp/plays/GenericPass.h"

#include <roboteam_utils/Tube.h>

#include "stp/invariants/BallClosestToUsInvariant.h"
#include "stp/invariants/BallOnOurSideInvariant.h"
#include "stp/invariants/FreedomOfRobotsInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "stp/roles/Formation.h"
#include "stp/roles/Halt.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/PassReceiver.h"
#include "stp/roles/Passer.h"

namespace rtt::ai::stp::play {

void GenericPass::onInitialize() noexcept {
    // Make sure we reset the passerShot flag
    passerShot = false;

    // Make sure we calculate pass positions at least once
    receiverPositionLeft = calculatePassLocation(gridLeft);
    receiverPositionRight = calculatePassLocation(gridRight);
    passingPosition = receiverPositionRight.first;
}

GenericPass::GenericPass() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallOnOurSideInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::FreedomOfRobotsInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::Passer>(role::Passer("passer")),
                                                                                 std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_left")),
                                                                                 std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_right")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("defender_1")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_3")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_4")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_5")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_6")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_7"))};
}

uint8_t GenericPass::score(world_new::World* world) noexcept { return 130; }

void GenericPass::calculateInfoForRoles() noexcept {
    auto ball = world->getWorld()->getBall()->get();

    /// Keeper
    stpInfos["keeper"].setPositionToShootAt(Vector2{0.0, 0.0});
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));

    /// Passer and receivers
    // calculate all info necessary to execute a pass
    calculateInfoForPass(ball);

    /// Defender
    auto enemyAttacker = world->getWorld()->getRobotClosestToBall(world_new::them);
    stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_1"].setEnemyRobot(enemyAttacker);
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    /// Midfielder
    if (stpInfos["midfielder_1"].getRobot()) {
        stpInfos["midfielder_1"].setAngle((ball->getPos() - stpInfos["midfielder_1"].getRobot()->get()->getPos()).angle());
    }
    auto fieldWidth = field.getFieldWidth();
    auto searchGrid = Grid(-0.15 * fieldWidth, -2, 0.10 * fieldWidth, 4, 4, 4);
    stpInfos["midfielder_1"].setPositionToMoveTo(control::ControlUtils::determineMidfielderPosition(searchGrid, field, world));
}

bool GenericPass::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap GenericPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag receiverFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"passer", {closeToBallFlag}});
    flagMap.insert({"receiver_left", {receiverFlag}});
    flagMap.insert({"receiver_right", {notImportant}});
    flagMap.insert({"midfielder_1", {notImportant}});
    flagMap.insert({"defender_1", {notImportant}});
    flagMap.insert({"halt_3", {notImportant}});
    flagMap.insert({"halt_4", {notImportant}});
    flagMap.insert({"halt_5", {notImportant}});
    flagMap.insert({"halt_6", {notImportant}});
    flagMap.insert({"halt_7", {notImportant}});

    return flagMap;
}

const char* GenericPass::getName() { return "Generic Pass"; }

void GenericPass::calculateInfoForPass(const world_new::ball::Ball* ball) noexcept {
    if (!passerShot && ball->getFilteredVelocity().length() > control_constants::BALL_STILL_VEL * 10) {
        passerShot = true;
    }

    bool passLeft{};
    Vector2 otherPos{};

    /// Recalculate pass positions if we did not shoot yet
    if (!passerShot) {
        /// For the receive locations, divide the field up into grids where the passers should stand,
        /// and find the best locations in those grids
        receiverPositionRight = calculatePassLocation(gridRight);
        receiverPositionLeft = calculatePassLocation(gridLeft);

        /// From the available receivers, select the best
        if (receiverPositionLeft.second > receiverPositionRight.second) {
            passingPosition = receiverPositionLeft.first;
            otherPos = receiverPositionRight.first;
            passLeft = true;
        } else {
            passingPosition = receiverPositionRight.first;
            otherPos = receiverPositionLeft.first;
            passLeft = false;
        }
    }
    /// Receiver should intercept when constraints are met
    if (passLeft && ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
        receiverPositionLeft.first = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
    } else if (ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
        receiverPositionRight.first = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
    }
    // Receiver
    stpInfos["receiver_left"].setPositionToMoveTo(receiverPositionLeft.first);
    stpInfos["receiver_right"].setPositionToMoveTo(receiverPositionRight.first);

    // decide kick or chip
    auto passLine = Tube(ball->getPos(), passingPosition, control_constants::ROBOT_CLOSE_TO_POINT / 2);
    auto allBots = world->getWorld()->getRobotsNonOwning();
    // For all bots except passer and receivers, check if they are on the pass line, aka robot should chip
    if (std::any_of(allBots.begin(), allBots.end(), [&](const auto& bot) {
      if ((stpInfos["passer"].getRobot() && bot->getId() == stpInfos["passer"].getRobot()->get()->getId()) ||
          (stpInfos["receiver_left"].getRobot() && bot->getId() == stpInfos["receiver_left"].getRobot()->get()->getId()) ||
          (stpInfos["receiver_right"].getRobot() && bot->getId() == stpInfos["receiver_right"].getRobot()->get()->getId())) {
          return false;
      }
      return passLine.contains(bot->getPos());
    })) {
        stpInfos["passer"].setKickOrChip(KickOrChip::CHIP);
    } else {
        stpInfos["passer"].setKickOrChip(KickOrChip::KICK);
    }
    // Passer
    stpInfos["passer"].setPositionToShootAt(passingPosition);
    stpInfos["passer"].setShotType(ShotType::PASS);
}

std::pair<Vector2, double> GenericPass::calculatePassLocation(Grid searchGrid) noexcept {
    double bestScore{};
    Vector2 bestPosition{};

    auto w = world->getWorld().value();
    auto fieldWidth = field.getFieldWidth();
    auto fieldLength = field.getFieldLength();
    auto fieldDiagonalLength = sqrt(fieldWidth * fieldWidth + fieldLength * fieldLength);

    auto ballPos = world->getWorld()->getBall()->get()->getPos();

    for (const auto& nestedPoints : searchGrid.getPoints()) {
        for (const auto& trial : nestedPoints) {
            // Make sure we only check valid points
            if (!FieldComputations::pointIsInDefenseArea(field, trial, false) && trial.dist(ballPos) > 2) {
                // Search closest bot to this point and get that distance
                auto theirClosestBot = w.getRobotClosestToPoint(trial, world_new::Team::them);
                auto theirClosestBotDistance{1.0};
                if (theirClosestBot) {
                    theirClosestBotDistance = theirClosestBot.value()->getPos().dist(trial) / fieldDiagonalLength;
                }
                // Calculate total score for this point
                auto pointScore = 0.5 * theirClosestBotDistance;

                // Check for best score
                if (pointScore > bestScore) {
                    bestScore = pointScore;
                    bestPosition = trial;
                }
            }
        }
    }
    /// If we can't reach target using kick, use chip
    auto passLine = Tube(w->getBall()->get()->getPos(), bestPosition, control_constants::ROBOT_CLOSE_TO_POINT / 2);
    auto enemyBots = w.getThem();
    if (std::any_of(enemyBots.begin(), enemyBots.end(), [&](const auto& bot) { return passLine.contains(bot->getPos()); })) {
        stpInfos["passer"].setKickOrChip(KickOrChip::CHIP);
    } else {
        stpInfos["passer"].setKickOrChip(KickOrChip::KICK);
    }
    return std::make_pair(bestPosition, bestScore);
}

bool GenericPass::isValidPlayToKeep(world_new::World* world) noexcept {
    world::Field field = world->getField().value();
    auto closestToBall = world->getWorld()->getRobotClosestToBall();
    auto canKeep = std::all_of(keepPlayInvariants.begin(), keepPlayInvariants.end(), [world, field](auto& x) { return x->checkInvariant(world->getWorld().value(), &field); }) &&
                   !passFinished();
    if (canKeep) {
        if (closestToBall && closestToBall->get()->getTeam() == world_new::us) {
            return true;
        } else if (world->getWorld()->getBall().value()->getVelocity().length() > control_constants::BALL_STILL_VEL) {
            return true;
        }
    }
    return false;
}

bool GenericPass::passFinished() noexcept {
    // TODO: improve this condition
    // Pass is done when one of the receivers is really close to the ball
    return (stpInfos["receiver_left"].getRobot() && stpInfos["receiver_left"].getRobot()->get()->getDistanceToBall() < 0.08) ||
           (stpInfos["receiver_right"].getRobot() && stpInfos["receiver_right"].getRobot()->get()->getDistanceToBall() < 0.08);
}
}  // namespace rtt::ai::stp::play
