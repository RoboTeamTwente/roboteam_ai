//
// Created by maxl on 09-02-21.
//

#include "stp/computations/PositionComputations.h"
#include <stp/computations/PassComputations.h>
#include "stp/StpInfo.h"

#include <roboteam_utils/Grid.h>

#include <include/roboteam_ai/world/Field.h>
#include "include/roboteam_ai/world/World.hpp"
#include "include/roboteam_ai/world/views/WorldDataView.hpp"

namespace rtt::ai::stp::computations {
    Vector2 PositionComputations::determineMidfielderPosition(const Grid &searchGrid, const rtt::world::Field &field,
                                                              rtt::world::World *world) {
        auto w = world->getWorld().value();
        double bestScore = 0;
        Vector2 bestPosition{};

        // From given grid, check the best point
        for (const auto &nestedPoints : searchGrid.getPoints()) {
            for (const auto &trial : nestedPoints) {
                // Make sure we only check valid points
                if (FieldComputations::pointIsValidPosition(field,trial)) {

                    // Search closest bot to this point and get that distance
                    auto theirClosestBot = w.getRobotClosestToPoint(trial, rtt::world::Team::them);
                    auto theirClosestBotDistance{1.0};
                    if (theirClosestBot) {
                        theirClosestBotDistance = theirClosestBot.value()->getPos().dist(trial);
                    }

                    // Calculate total score for this point
                    auto pointScore = theirClosestBotDistance;

                    // Check for best score
                    if (pointScore > bestScore) {
                        bestScore = pointScore;
                        bestPosition = trial;
                    }
                }
            }
        }
        return bestPosition;
    };

    Vector2 PositionComputations::determineOpenPosition(const Grid &searchGrid, const rtt::world::Field &field,
                                                              rtt::world::World *world, bool clearPath, double shotMargin) {
        auto w = world->getWorld().value();
        double bestScore = 0;
        Vector2 bestPosition{};

        // From given grid, check the best point
        for (const auto &nestedPoints : searchGrid.getPoints()) {
            for (const auto &trial : nestedPoints) {
                // Make sure we only check valid points
                if (FieldComputations::pointIsValidPosition(field,trial)) {
                    // If path to the point has to be clear, then skip if a point is not
                    if (clearPath) {
                        auto passLine = Tube(w->getBall()->get()->getPos(), bestPosition, shotMargin);
                        if (computations::PassComputations::pathHasAnyRobots(passLine, world->getWorld()->getRobotsNonOwning())) continue;
                    }
                    // Search closest bot to this point and get that distance
                    auto theirClosestBot = w.getRobotClosestToPoint(trial, rtt::world::Team::them);
                    auto theirClosestBotDistance{1.0};
                    if (theirClosestBot) {
                        theirClosestBotDistance = theirClosestBot.value()->getPos().dist(trial);
                    }

                    // Calculate total score for this point
                    auto pointScore = theirClosestBotDistance;

                    // Check for best score
                    if (pointScore > bestScore) {
                        bestScore = pointScore;
                        bestPosition = trial;
                    }
                }
            }
        }
        return bestPosition;
    };

//    void PositionComputations::determineGoalShootLocation(const world::ball::Ball* ball) noexcept {
//        bool passLeft{};
//        Vector2 otherPos{};
//
//        /// Recalculate pass positions if we did not shoot yet
//            /// For the receive locations, divide the field up into grids where the passers should stand,
//            /// and find the best locations in those grids
//            receiverPositionRight = calculatePassLocation(gridRight);
//            receiverPositionLeft = calculatePassLocation(gridLeft);
//
//            /// From the available receivers, select the best
//            if (receiverPositionLeft.second > receiverPositionRight.second) {
//                passingPosition = receiverPositionLeft.first;
//                otherPos = receiverPositionRight.first;
//                passLeft = true;
//            } else {
//                passingPosition = receiverPositionRight.first;
//                otherPos = receiverPositionLeft.first;
//                passLeft = false;
//            }
//        /// Receiver should intercept when constraints are met
//        if (passLeft && ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
//            receiverPositionLeft.first = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
//        } else if (ball->getVelocity().length() > control_constants::HAS_KICKED_ERROR_MARGIN) {
//            receiverPositionRight.first = Line(ball->getPos(), ball->getPos() + ball->getFilteredVelocity()).project(passingPosition);
//        }
//        // Receiver
//        stpInfos["receiver_left"].setPositionToMoveTo(receiverPositionLeft.first);
//        stpInfos["receiver_right"].setPositionToMoveTo(receiverPositionRight.first);
//
//        // decide kick or chip
//        auto passLine = Tube(ball->getPos(), passingPosition, control_constants::ROBOT_CLOSE_TO_POINT / 2);
//        auto allBots = world->getWorld()->getRobotsNonOwning();
//        // For all bots except passer and receivers, check if they are on the pass line, aka robot should chip
//        if (std::any_of(allBots.begin(), allBots.end(), [&](const auto& bot) {
//            if ((stpInfos["passer"].getRobot() && bot->getId() == stpInfos["passer"].getRobot()->get()->getId()) ||
//                (stpInfos["receiver_left"].getRobot() && bot->getId() == stpInfos["receiver_left"].getRobot()->get()->getId()) ||
//                (stpInfos["receiver_right"].getRobot() && bot->getId() == stpInfos["receiver_right"].getRobot()->get()->getId())) {
//                return false;
//            }
//            return passLine.contains(bot->getPos());
//        })) {
//            stpInfos["passer"].setKickOrChip(KickOrChip::CHIP);
//        } else {
//            stpInfos["passer"].setKickOrChip(KickOrChip::KICK);
//        }
//        // Passer
//        stpInfos["passer"].setPositionToShootAt(passingPosition);
//        stpInfos["passer"].setShotType(ShotType::TARGET);
//    }

    std::pair<Vector2, double> PositionComputations::determineGoalShotLocation(const Grid &searchGrid, const rtt::world::Field &field,
                                                                           rtt::world::World *world) {
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
                    auto theirClosestBot = w.getRobotClosestToPoint(trial, world::Team::them);
                    auto theirClosestBotDistance{1.0};
                    if (theirClosestBot) {
                        theirClosestBotDistance = theirClosestBot.value()->getPos().dist(trial) / fieldDiagonalLength;
                    }
                    // Calculate total score for this point
                    // TODO-Max check score factors to be logical
                    // TODO-Max add difference between chip and kick
                    auto pointScore = (goalDistance + visibility + trialToGoalAngle) * (0.5 * theirClosestBotDistance);

                    // Check for best score
                    if (pointScore > bestScore) {
                        bestScore = pointScore;
                        bestPosition = trial;
                    }
                }
            }
        }
        return std::make_pair(bestPosition, bestScore);
    }
}; //namespace computations