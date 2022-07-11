//
// Created by timovdk on 5/20/20.
/// TODO Needs to be refactored to use new passComputations (or play should be removed)
//

#include "stp/plays/offensive/GenericPass.h"

#include <roboteam_utils/Tube.h>

#include "stp/computations/PositionComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/passive/Halt.h"

namespace rtt::ai::stp::play {

void GenericPass::onInitialize() noexcept {
    // Make sure we reset the passerShot flag
    passerShot = false;

    // Make sure we calculate pass positions at least once
    receiverPositionLeft = PositionComputations::getPosition(stpInfos["receiver_left"].getPositionToMoveTo(), field.getFrontLeftGrid(), gen::GoalShootPosition, field, world);
    receiverPositionRight = PositionComputations::getPosition(stpInfos["receiver_right"].getPositionToMoveTo(), field.getFrontRightGrid(), gen::GoalShootPosition, field, world);
    passingPosition = receiverPositionRight.position;
}

GenericPass::GenericPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::BallOnOurSide);
    startPlayEvaluation.emplace_back(eval::BallClosestToUs);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::FreedomOfRobots);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::Passer>(role::Passer("passer")),
                                                                                 std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_left")),
                                                                                 std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_right")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                 std::make_unique<role::BallDefender>(role::BallDefender("defender_1")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_3")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_4")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_5")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_6")),
                                                                                 std::make_unique<role::Halt>(role::Halt("halt_7"))};
}

uint8_t GenericPass::score(const rtt::world::Field& field) noexcept { return 130; }

void GenericPass::calculateInfoForRoles() noexcept {
    auto ball = world->getWorld()->getBall()->get();

    /// Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    /// Passer and receivers
    // calculate all info necessary to execute a pass
    calculateInfoForPass(ball);

    /// BallDefender
    auto enemyAttacker = world->getWorld()->getRobotClosestToBall(world::them);
    stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_1"].setEnemyRobot(enemyAttacker);
    stpInfos["defender_1"].setBlockDistance(BlockDistance::CLOSE);

    /// Midfielder
    if (stpInfos["midfielder_1"].getRobot()) {
        stpInfos["midfielder_1"].setAngle((ball->position - stpInfos["midfielder_1"].getRobot()->get()->getPos()).angle());
    }
    auto fieldWidth = field.getFieldWidth();
    auto searchGrid = Grid(-0.15 * fieldWidth, -2, 0.10 * fieldWidth, 4, 4, 4);
    // TODO: check if SafePosition is the right profile to use
    stpInfos["midfielder_1"].setPositionToMoveTo(
        PositionComputations::getPosition(stpInfos["midfielder_1"].getPositionToMoveTo(), field.getMiddleMidGrid(), gen::SafePosition, field, world));
}

Dealer::FlagMap GenericPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag receiverFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"passer", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
    flagMap.insert({"receiver_left", {DealerFlagPriority::REQUIRED, {receiverFlag}}});
    flagMap.insert({"receiver_right", {DealerFlagPriority::REQUIRED, {receiverFlag}}});
    flagMap.insert({"midfielder_1", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {notImportant}}});
    flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
    flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});

    return flagMap;
}

const char* GenericPass::getName() { return "Generic Pass"; }

void GenericPass::calculateInfoForPass(const world::ball::Ball* ball) noexcept {
    if (!passerShot && ball->velocity.length() > control_constants::BALL_STILL_VEL * 10) {
        passerShot = true;
    }

    bool passLeft{};
    Vector2 otherPos{};

    /// Recalculate pass positions if we did not shoot yet
    if (!passerShot) {
        /// For the receive locations, divide the field up into grids where the passers should stand,
        /// and find the best locations in those grids
        receiverPositionLeft = PositionComputations::getPosition(stpInfos["receiver_left"].getPositionToMoveTo(), field.getFrontLeftGrid(), gen::GoalShootPosition, field, world);
        receiverPositionRight =
            PositionComputations::getPosition(stpInfos["receiver_right"].getPositionToMoveTo(), field.getFrontRightGrid(), gen::GoalShootPosition, field, world);

        /// From the available receivers, select the best
        if (receiverPositionLeft.score > receiverPositionRight.score) {
            passingPosition = receiverPositionLeft.position;
            otherPos = receiverPositionRight.position;
            passLeft = true;
        } else {
            passingPosition = receiverPositionRight.position;
            otherPos = receiverPositionLeft.position;
            passLeft = false;
        }
    }
    /// Receiver should intercept when constraints are met
    if (passLeft && ball->velocity.length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
        receiverPositionLeft.position = Line(ball->position, ball->position + ball->velocity).project(passingPosition);
    } else if (ball->velocity.length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
        receiverPositionRight.position = Line(ball->position, ball->position + ball->velocity).project(passingPosition);
    }
    // Receiver
    stpInfos["receiver_left"].setPositionToMoveTo(receiverPositionLeft.position);
    stpInfos["receiver_right"].setPositionToMoveTo(receiverPositionRight.position);

    // decide kick or chip
    auto passLine = Tube(ball->position, passingPosition, control_constants::ROBOT_CLOSE_TO_POINT / 2);
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
    stpInfos["passer"].setPositionToShootAt(field.getTheirGoalCenter());
    stpInfos["passer"].setShotType(ShotType::PASS);
}

bool GenericPass::passFinished() noexcept {
    // TODO: improve this condition
    // Pass is done when one of the receivers is really close to the ball
    return (stpInfos["receiver_left"].getRobot() && stpInfos["receiver_left"].getRobot()->get()->getDistanceToBall() < 0.08) ||
           (stpInfos["receiver_right"].getRobot() && stpInfos["receiver_right"].getRobot()->get()->getDistanceToBall() < 0.08);
}
}  // namespace rtt::ai::stp::play
