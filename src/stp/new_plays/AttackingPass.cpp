//
// Created by jessevw on 17.03.20.
//

#include "stp/new_plays/AttackingPass.h"

#include <stp/new_roles/Halt.h>
#include <stp/new_roles/Keeper.h>
#include <include/roboteam_ai/stp/new_roles/Formation.h>
#include "stp/invariants/BallClosestToUsInvariant.h"

#include "roboteam_utils/Grid.h"
#include "roboteam_utils/Tube.h"
#include "stp/invariants/BallCloseToUsInvariant.h"
#include "stp/invariants/BallMovesSlowInvariant.h"
#include "stp/invariants/NoGoalVisionFromBallInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "stp/new_roles/PassReceiver.h"
#include "stp/new_roles/Passer.h"

namespace rtt::ai::stp::play {

void AttackingPass::onInitialize() noexcept {
    passerShot = false;
    receiverPositionLeft = calculatePassLocation(gridLeft);
    receiverPositionRight = calculatePassLocation(gridRight);
    passingPosition = receiverPositionRight.first;
}

AttackingPass::AttackingPass() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToUsInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::NoGoalVisionFromBallInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    //keepPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                       std::make_unique<role::Passer>(role::Passer("passer")),
                                                                                       std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_left")),
                                                                                       std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_right")),
                                                                                       std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                       std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_5")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_6")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_7")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_8")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_9"))};
}

uint8_t AttackingPass::score(world_new::World* world) noexcept { return 50; }

Dealer::FlagMap AttackingPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag passerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag receiverFlag(DealerFlagTitle::WITH_WORKING_DRIBBLER, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"passer", {passerFlag}});
    flagMap.insert({"receiver_left", {receiverFlag}});
    flagMap.insert({"receiver_right", {receiverFlag}});
    flagMap.insert({"midfielder_1", {not_important}});
    //flagMap.insert({"midfielder_2", {not_important}});
    /*flagMap.insert({"test_role_5", {not_important}});
    flagMap.insert({"test_role_6", {not_important}});
    flagMap.insert({"test_role_7", {not_important}});
    flagMap.insert({"test_role_8", {not_important}});
    flagMap.insert({"test_role_9", {not_important}});*/

    return flagMap;
}

void AttackingPass::calculateInfoForRoles() noexcept {
    auto ball = world->getWorld()->getBall()->get();

    if(!passerShot && ball->getFilteredVelocity().length() > control_constants::BALL_STILL_VEL * 10) {
        passerShot = true;
    }
/*
    else if (passerShot && stpInfos["passer"].getRobot() && stpInfos["passer"].getRobot()->get()->getPos().dist(ball->getPos()) < 0.2) {
        passerShot = false;
    }
*/

    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    // Calculate most important positions to defend
    // You know you have n defenders, because the play assigned it that way
    auto enemyRobots = world->getWorld()->getThem();
    const int numberOfDefenders = 1;
    auto defensivePositions = calculateDefensivePositions(numberOfDefenders, world, enemyRobots);

    /// For the receive locations, divide the field up into grids where the passers should stand,
    /// and find the best locations in those grids

    /// From the available receivers, select the best
    bool passLeft{};
    Vector2 otherpos{};

    /// Recalculate thing
    if(!passerShot) {
        receiverPositionRight = calculatePassLocation(gridRight);
        receiverPositionLeft = calculatePassLocation(gridLeft);

        if (receiverPositionLeft.second > receiverPositionRight.second) {
            passingPosition = receiverPositionLeft.first;
            otherpos = receiverPositionRight.first;
            passLeft = true;
        } else {
            passingPosition = receiverPositionRight.first;
            otherpos = receiverPositionLeft.first;
            passLeft = false;
        }
    }
    ai::interface::Input::drawData(ai::interface::Visual::BALL_DATA, {passingPosition}, ai::Constants::ROBOT_COLOR_BLUE(), -1, ai::interface::Drawing::CIRCLES, 80, 80, 6);
    ai::interface::Input::drawData(ai::interface::Visual::BALL_DATA, {otherpos}, ai::Constants::SELECTED_ROBOT_COLOR(), -1, ai::interface::Drawing::CIRCLES, 80, 80, 6);


    if (passLeft && ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
        receiverPositionLeft.first = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
    } else if (ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
        receiverPositionRight.first = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
    }
    ai::interface::Input::drawData(ai::interface::Visual::BALL_DATA, {receiverPositionLeft.first}, ai::Constants::ROBOT_COLOR_BLUE(), -1, ai::interface::Drawing::CIRCLES, 40, 40, 6);
    ai::interface::Input::drawData(ai::interface::Visual::BALL_DATA, {receiverPositionRight.first}, ai::Constants::SELECTED_ROBOT_COLOR(), -1, ai::interface::Drawing::CIRCLES, 40, 40, 6);
    // Receiver
    stpInfos["receiver_left"].setPositionToMoveTo(receiverPositionLeft.first);
    stpInfos["receiver_right"].setPositionToMoveTo(receiverPositionRight.first);

    // decide kick or chip
    auto passLine = Tube(ball->getPos(), passingPosition, control_constants::ROBOT_CLOSE_TO_POINT/2);
    auto allBots = world->getWorld()->getRobotsNonOwning();
    if (std::any_of(allBots.begin(), allBots.end(), [&](const auto& bot) {
        if((stpInfos["passer"].getRobot() && bot->getId() == stpInfos["passer"].getRobot()->get()->getId())
        || (stpInfos["receiver_left"].getRobot() && bot->getId() == stpInfos["receiver_left"].getRobot()->get()->getId())
        || (stpInfos["receiver_right"].getRobot() && bot->getId() == stpInfos["receiver_right"].getRobot()->get()->getId())) {
            return false;
        }
        return passLine.contains(bot->getPos());
    })) {
        stpInfos["passer"].setShootType(CHIP);
    } else {
        stpInfos["passer"].setShootType(KICK);
    }

    // Passer
    stpInfos["passer"].setPositionToShootAt(passingPosition);
    stpInfos["passer"].setKickChipType(TARGET);

    // Defenders
    for (int defenderIndex = 0; defenderIndex < numberOfDefenders; defenderIndex++) {
        std::string defenderName = "defender" + std::to_string(defenderIndex + 1);

        if (stpInfos.find(defenderName) != stpInfos.end()) {
            stpInfos[defenderName].setPositionToMoveTo(defensivePositions[defenderIndex]);
        }
    }

    if (stpInfos["midfielder_1"].getRobot() && stpInfos["midfielder_2"].getRobot()) {
        stpInfos["midfielder_2"].setAngle((ball->getPos() - stpInfos["midfielder_2"].getRobot()->get()->getPos()).angle());
        stpInfos["midfielder_1"].setAngle((ball->getPos() - stpInfos["midfielder_1"].getRobot()->get()->getPos()).angle());
    }
    auto fieldWidth = field.getFieldWidth();

    auto searchGridLeft = Grid(-0.15*fieldWidth, 0, 0.25*fieldWidth, 1.5, 3, 3);
    auto searchGridRight = Grid(-0.25*fieldWidth, -1.5, 0.25*fieldWidth, 1.5, 3, 3);
    stpInfos["midfielder_1"].setPositionToMoveTo(control::ControlUtils::determineMidfielderPosition(searchGridRight, field, world));
    stpInfos["midfielder_2"].setPositionToMoveTo(control::ControlUtils::determineMidfielderPosition(searchGridLeft, field, world));

}

std::vector<Vector2> AttackingPass::calculateDefensivePositions(int numberOfDefenders, world_new::World* world, std::vector<world_new::view::RobotView> enemyRobots) {
    std::vector<Vector2> positions = {};

    // 3 robots will defend goal
    for (int i = 0; i < numberOfDefenders; i++) {
        if (i < 3) {
            positions.push_back(world->getField()->getOurGoalCenter());
        } else {
            positions.push_back(enemyRobots[i].get()->getPos());
        }
    }

    return positions;
}

bool AttackingPass::shouldRoleSkipEndTactic() { return false; }

const char* AttackingPass::getName() { return "AttackingPass"; }

std::pair<Vector2, double> AttackingPass::calculatePassLocation(Grid searchGrid) noexcept{
    auto fieldWidth = field.getFieldWidth();
    auto fieldLength = field.getFieldLength();

    double bestScore = 0;
    Vector2 bestPosition{};

    auto w = world->getWorld().value();
    auto ballPos = w.getBall().value()->getPos();

    // Make a grid with all potentially good points
    for (const auto& nestedPoints : searchGrid.getPoints()) {
        for (const auto& trial : nestedPoints) {
            // Make sure we only check valid points
            if (!FieldComputations::pointIsInDefenseArea(field, trial, false) && trial.dist(ballPos) > 2) {
                // Check goal visibility from  a point
                auto visibility = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, false, trial, w) / 100;

                // Normalize distance, and then subtract 1
                // This inverts the score, so if the distance is really large,
                // the score for the distance will be close to 0
                auto fieldDiagonalLength = sqrt(fieldWidth * fieldWidth + fieldLength * fieldLength);
                auto goalDistance = 1 - (FieldComputations::getDistanceToGoal(field, false, trial) / fieldDiagonalLength);

                // Make sure the angle to shoot at the goal with is okay
                auto trialToGoalAngle = 1 - fabs((field.getTheirGoalCenter() - trial).angle()) / M_PI_2;

                // Search closest bot to this point and get that distance
                auto theirClosestBot = w.getRobotClosestToPoint(trial, world_new::Team::them);
                auto theirClosestBotDistance{1.0};
                if (theirClosestBot) {
                    theirClosestBotDistance = theirClosestBot.value()->getPos().dist(trial) / fieldDiagonalLength;
                }

                // Calculate total score for this point
                auto pointScore = (goalDistance + visibility + trialToGoalAngle) * (0.5 * theirClosestBotDistance);

                // Check for best score
                if (pointScore > bestScore) {
                    bestScore = pointScore;
                    bestPosition = trial;
                }
            }
        }
    }
    /// If we can't reach target using kick, use chip
    return std::make_pair(bestPosition, bestScore);
}

bool AttackingPass::isValidPlayToKeep(world_new::World* world) noexcept {
    world::Field field = world->getField().value();
    auto closestToBall = world->getWorld()->getRobotClosestToBall();
    auto canKeep = std::all_of(keepPlayInvariants.begin(), keepPlayInvariants.end(), [world, field](auto& x) { return x->checkInvariant(world->getWorld().value(), &field); }) &&
            !passFinished();
    if (canKeep) {
        if (closestToBall && closestToBall->get()->getTeam() == world_new::us) {
            return true;
        }
        else {
            if (world->getWorld()->getBall().value()->getVelocity().length() > control_constants::BALL_STILL_VEL) {
                return true;
            }
        }
    }
    return false;

}

bool AttackingPass::passFinished() noexcept {
    // TODO: fix this condition
    return (stpInfos["receiver_left"].getRobot() && stpInfos["receiver_left"].getRobot()->get()->getDistanceToBall() < 0.08)
    || (stpInfos["receiver_right"].getRobot() && stpInfos["receiver_right"].getRobot()->get()->getDistanceToBall() < 0.08);
}

bool AttackingPass::passFailed() noexcept {
    // TODO: fix this condition
    return (stpInfos["receiver_left"].getRobot() && stpInfos["receiver_left"].getRobot()->get()->getAngleDiffToBall() > M_PI_4)
    && (stpInfos["receiver_right"].getRobot() && stpInfos["receiver_right"].getRobot()->get()->getAngleDiffToBall() > M_PI_4);
}
}  // namespace rtt::ai::stp::play
