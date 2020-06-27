//
// Created by jessevw on 17.03.20.
//

#include "stp/new_plays/AttackingPass.h"

#include <stp/new_roles/Halt.h>
#include <stp/new_roles/Keeper.h>
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
    passingPosition = calculatePassLocation(Grid(0.15*field.getFieldWidth(), -2.5, 2 , 2.5, 5, 5)).first;
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
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_3")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_4")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_5")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_6")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_7")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_8")),
                                                                                       std::make_unique<role::Halt>(role::Halt("test_role_9"))};
}

uint8_t AttackingPass::score(world_new::World* world) noexcept { return 131; }

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
    flagMap.insert({"test_role_3", {not_important}});
    flagMap.insert({"test_role_4", {not_important}});
    /*flagMap.insert({"test_role_5", {not_important}});
    flagMap.insert({"test_role_6", {not_important}});
    flagMap.insert({"test_role_7", {not_important}});
    flagMap.insert({"test_role_8", {not_important}});
    flagMap.insert({"test_role_9", {not_important}});*/

    return flagMap;
}

void AttackingPass::calculateInfoForRoles() noexcept {
    auto ball = world->getWorld()->getBall()->get();

    if(!passerShot && ball->getVelocity().length() > control_constants::BALL_STILL_VEL * 10) {
        passerShot = true;
    }

    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    // Calculate most important positions to defend
    // You know you have n defenders, because the play assigned it that way
    auto enemyRobots = world->getWorld()->getThem();
    const int numberOfDefenders = 1;
    auto defensivePositions = calculateDefensivePositions(numberOfDefenders, world, enemyRobots);

    /// For the receive locations, divide the field up into grids where the passers should stand,
    /// and find the best locations in those grids
    auto gridLeft = Grid(0.15*field.getFieldWidth(), 0, 2, 2.5, 5, 5);
    auto gridRight = Grid(0.15*field.getFieldWidth(), -2.5, 2 , 2.5, 5, 5);

    auto [receiverPositionRight, scoreRight] = calculatePassLocation(gridRight);
    auto [receiverPositionLeft, scoreLeft] = calculatePassLocation(gridLeft);

    /// From the available receivers, select the best
    bool passLeft{};
    Vector2 otherpos{};
    if(!passerShot) {
        if (scoreLeft > scoreRight) {
            passingPosition = receiverPositionLeft;
            otherpos = receiverPositionRight;
            passLeft = true;
        }
        else {
            passingPosition = receiverPositionRight;
            otherpos = receiverPositionLeft;
            passLeft = false;
        }
    }
    ai::interface::Input::drawData(ai::interface::Visual::BALL_DATA, {passingPosition}, ai::Constants::ROBOT_COLOR_BLUE(), -1, ai::interface::Drawing::CIRCLES, 80, 80, 6);
    ai::interface::Input::drawData(ai::interface::Visual::BALL_DATA, {otherpos}, ai::Constants::SELECTED_ROBOT_COLOR(), -1, ai::interface::Drawing::CIRCLES, 80, 80, 6);

    if (passLeft && ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
        receiverPositionLeft = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
    }

    if (!passLeft && ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
        receiverPositionRight = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
    }
    // Receiver
    stpInfos["receiver_left"].setPositionToMoveTo(receiverPositionLeft);
    stpInfos["receiver_right"].setPositionToMoveTo(receiverPositionRight);

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
            if (!FieldComputations::pointIsInDefenseArea(field, trial, false) && trial.dist(ballPos) > 1) {
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
    auto passLine = Tube(w->getBall()->get()->getPos(), bestPosition, control_constants::ROBOT_CLOSE_TO_POINT/2);
    auto enemyBots = w.getThem();
    if (std::any_of(enemyBots.begin(), enemyBots.end(), [&](const auto& bot) { return passLine.contains(bot->getPos()); })) {
        stpInfos["passer"].setShootType(CHIP);
    } else {
        stpInfos["passer"].setShootType(KICK);
    }
    return std::make_pair(bestPosition, bestScore);
}

bool AttackingPass::isValidPlayToKeep(world_new::World* world) noexcept {
    world::Field field = world->getField().value();
    auto closestToBall = world->getWorld()->getRobotClosestToBall();
    if (closestToBall && closestToBall->get()->getTeam() == world_new::us) {
        return true;
    }
    else {
        if (world->getWorld()->getBall().value()->getVelocity().length() > control_constants::BALL_STILL_VEL) {
            return true;
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
