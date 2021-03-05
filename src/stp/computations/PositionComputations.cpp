//
// Created by maxl on 09-02-21.
// TODO-Max Make profiles for each position/situation that is using in BestLocation.
    /// IDEA: If more factor are needed, the profile (with the factors of each statement) can be adjusted without comprimising the plays
    ///       A profile for midfielder could be that the vision of the goal is not important but the line of isght is (for a pass)
// TODO-Max remove determine..Position functions that are no longer used.

#include "stp/computations/PositionComputations.h"
#include <stp/computations/PassComputations.h>
#include "stp/StpInfo.h"

#include <roboteam_utils/Grid.h>

#include <include/roboteam_ai/world/Field.h>
#include <roboteam_utils/Circle.h>
#include "include/roboteam_ai/world/World.hpp"

namespace rtt::ai::stp::computations {
    std::pair<Vector2, double> PositionComputations::determineBestOpenPosition(const Grid &searchGrid, const rtt::world::Field &field,
                                                                               rtt::world::World *world) {
        double bestScore = 0;
        Vector2 bestPosition{};

        // From given grid, check the best point
        for (const auto &nestedPoints : searchGrid.getPoints()) {
            for (const auto &trial : nestedPoints) {
                // Make sure we only check valid points
                if (FieldComputations::pointIsValidPosition(field,trial)) {

                    // Calculate total score for this point
                    auto pointScore = determineOpenScore(trial,world);

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

    std::pair<Vector2, double> PositionComputations::determineBestLineOfSightPosition(const Grid &searchGrid, const rtt::world::Field &field,
                                                                                      rtt::world::World *world) {
        double bestScore = 0;
        Vector2 bestPosition{};

        // From given grid, check the best point
        for (const auto &nestedPoints : searchGrid.getPoints()) {
            for (const auto &trial : nestedPoints) {
                // Make sure we only check valid points
                if (FieldComputations::pointIsValidPosition(field, trial)) {
                    auto pointScore = determineLineOfSightScore(trial, world);
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

    std::pair<Vector2, double> PositionComputations::determineBestGoalShotLocation(const Grid &searchGrid, const rtt::world::Field &field,
                                                                                   rtt::world::World *world) {
        double bestScore = 0;
        Vector2 bestPosition{};

        auto w = world->getWorld().value();
        auto ballPos = w.getBall().value()->getPos();

        // Make a grid with all potentially good points
        for (const auto& nestedPoints : searchGrid.getPoints()) {
            for (const auto& trial : nestedPoints) {
                // Make sure we only check valid points
                if (!FieldComputations::pointIsInDefenseArea(field, trial, false) && trial.dist(ballPos) > 2) {
                    auto pointScore = determineGoalShotScore(trial, field, world);
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

    std::pair<Vector2, double> PositionComputations::determineBestLocation(const Grid &searchGrid, const rtt::world::Field &field,
                                                                                   rtt::world::World *world, int factorOpen, int factorLineOfSight, int factorVisionGoal) {
        double bestScore = 0;
        Vector2 bestPosition{};

        // From given grid, check the best point
        for (const auto &nestedPoints : searchGrid.getPoints()) {
            for (const auto &trial : nestedPoints) {
                // Make sure we only check valid points
                double pointScore = 0;
                if (FieldComputations::pointIsValidPosition(field, trial)) {
                    pointScore += determineOpenScore(trial,world) * factorOpen;
                    pointScore += determineLineOfSightScore(trial,world) * factorLineOfSight;
                    pointScore += determineGoalShotScore(trial,field,world) * factorVisionGoal;
                }
                if (pointScore > bestScore) {
                    bestScore = pointScore;
                    bestPosition = trial;
                }
            }
        }
        return std::make_pair(bestPosition, bestScore);
    }

    double PositionComputations::determineOpenScore(Vector2 point, rtt::world::World *world) {
        /// TODO-Max determine with all robots within a circle around
        /// TODO-Max maybe have a cone that aims at the ball (behind does not really matter?)
        double score = 0;
        auto theirClosestBot = world->getWorld().value().getRobotClosestToPoint(point, rtt::world::Team::them);
        if (theirClosestBot) {
            score = theirClosestBot.value()->getPos().dist(point);
        }
        return score;
    }

    double PositionComputations::determineLineOfSightScore(Vector2 point, rtt::world::World *world) {
        /// TODO-Max make triangle
        auto passLine = Line(world->getWorld().value()->getBall()->get()->getPos(), point);
        double score = 0;
        // If there is a robot in the way, return 0
        if (computations::PassComputations::pathHasAnyRobots(passLine, world->getWorld()->getRobotsNonOwning())) return score;
        /// TODO-Max make it this better, now takes center of line
        // Search closest bot to this point and get that distance
        auto theirClosestBot = world->getWorld().value().getRobotClosestToPoint((passLine.v1+passLine.v2)/2, rtt::world::Team::them);
        if (theirClosestBot) {
            score = theirClosestBot.value()->getPos().dist(point);
        }
        return score;
    }

    double PositionComputations::determineGoalShotScore(Vector2 point, const rtt::world::Field &field, rtt::world::World *world) {
        auto fieldWidth = field.getFieldWidth();
        auto fieldLength = field.getFieldLength();
        auto w = world->getWorld().value();
        auto visibility = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, false, point, w) / 100;

        // Normalize distance, and then subtract 1
        // This inverts the score, so if the distance is really large,
        // the score for the distance will be close to 0
        auto fieldDiagonalLength = sqrt(fieldWidth * fieldWidth + fieldLength * fieldLength);
        auto goalDistance = 1 - (FieldComputations::getDistanceToGoal(field, false, point) / fieldDiagonalLength);

        // Make sure the angle to shoot at the goal with is okay
        auto trialToGoalAngle = 1 - fabs((field.getTheirGoalCenter() - point).angle()) / M_PI_2;

        // Calculate total score for this point
        // TODO-Max check score factors to be logical
        // TODO-Max add difference between chip and kick
        double score = (goalDistance + visibility + trialToGoalAngle);
        return score;
    }

    std::vector<Vector2> PositionComputations::determineWallPositions(const rtt::world::Field &field, rtt::world::World *world, int amountDefenders) {
        auto w = world->getWorld().value();
        auto b = w.getBall()->get();
        double spacingRobots = control_constants::ROBOT_RADIUS*2;

        std::vector<Vector2> positions = {};
        std::vector<LineSegment> defenseAreaBorder = FieldComputations::getDefenseArea(field, true,
                                                                                       control_constants::ROBOT_RADIUS +
                                                                                       control_constants::GO_TO_POS_ERROR_MARGIN,
                                                                                       0).getBoundary();
        defenseAreaBorder.erase(defenseAreaBorder.begin() + 2);       // Ugly but the back boarder is not needed.
        for (auto &i : defenseAreaBorder) {
            ///RTT_DEBUG(std::to_string(i.start.x) + "," + std::to_string(i.start.y) + " TO " + std::to_string(i.end.x) + "," +std::to_string(i.end.y) + "  .");
            interface::Input::drawData(interface::Visual::DEBUG,std::vector({Vector2(i.start.x,i.start.y),Vector2(i.end.x,i.end.y)}),Qt::blue,-1,interface::Drawing::LINES_CONNECTED);
            LineSegment ball2GoalLine = LineSegment(b->getPos(), field.getOurGoalCenter());
            std::vector<Vector2> lineBorderIntersects = {};
            /// Vector is made to check if there is only 1 intersect
            for (const LineSegment &line : defenseAreaBorder) {
                if (line.doesIntersect(ball2GoalLine)) {
                    auto intersect = line.intersects(ball2GoalLine);
                    if (intersect.has_value()) lineBorderIntersects.push_back(intersect.value());
                }
            }

            // Always use the first (as there should only be one.
            Vector2 lineBorderIntersect = lineBorderIntersects.front();

            interface::Input::drawData(interface::Visual::DEBUG,std::vector({Vector2(ball2GoalLine.start.x,ball2GoalLine.start.y),Vector2(ball2GoalLine.end.x,ball2GoalLine.end.y)}),Qt::blue,-1,interface::Drawing::LINES_CONNECTED);
            interface::Input::drawData(interface::Visual::DEBUG,std::vector({lineBorderIntersect}),Qt::red,-1,interface::Drawing::CIRCLES, 3 ,3, 2);

            // DEBUG
            if (lineBorderIntersects.empty() || lineBorderIntersects.size() > 1) {
                //RTT_DEBUG("determineWallPositions broke. Size is " + std::to_string(lineBorderIntersects.size()) + "!...");
            } else {
                //RTT_DEBUG("Intersect is at " + std::to_string(lineBorderIntersect.x) + "," +std::to_string(lineBorderIntersect.y));
                int i = 1;
                if (amountDefenders % 2) {
                    /// ODD
                    positions.push_back(lineBorderIntersect);
                    while (positions.size() < amountDefenders) {
                        auto circle = Circle(lineBorderIntersect, (i++) * (spacingRobots));
                        for (const LineSegment &line : defenseAreaBorder) {
                            auto intersects = circle.intersectsCircleWithLineSegment(circle, line);
                            for (auto intersect : intersects) {
                                positions.push_back(intersect);
                            }
                        }
                    }
                } else {
                    /// EVEN
                    while (positions.size() < amountDefenders) {
                        auto circle = Circle(lineBorderIntersect, (-0.5+i++) * (spacingRobots));
                        for (const LineSegment &line : defenseAreaBorder) {
                            auto intersects = circle.intersectsCircleWithLineSegment(circle, line);
                            for (auto intersect : intersects) {
                                positions.push_back(intersect);
                            }
                        }
                    }
                }
            }
        }
        return positions;
    }
} //namespace computations