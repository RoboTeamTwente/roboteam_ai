//
// Created by maxl on 09-02-21.
//

#include "stp/computations/PositionComputations.h"
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
                        if (FieldComputations::pathHasAnyRobots(passLine, world->getWorld()->getRobotsNonOwning())) continue;
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
}; //namespace computations