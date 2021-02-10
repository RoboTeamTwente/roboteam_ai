//
// Created by maxl on 09-02-21.
//

#include "stp/computations/PositionComputations.h"
#include <roboteam_utils/Grid.h>
#include <include/roboteam_ai/world/Field.h>
#include "stp/StpInfo.h"
#include "include/roboteam_ai/world/World.hpp"

namespace rtt::ai::stp::computations {
    Vector2 PositionComputations::determineMidfielderPosition(const Grid &searchGrid, const rtt::world::Field &field,
                                                              rtt::world::World *world) {
        auto fieldWidth = field.getFieldWidth();
        auto fieldLength = field.getFieldLength();

        auto w = world->getWorld().value();

        double bestScore = 0;
        Vector2 bestPosition{};

        // Make a grid with all potentially good points
        for (const auto &nestedPoints : searchGrid.getPoints()) {
            for (const auto &trial : nestedPoints) {
                // Make sure we only check valid points
                if (!FieldComputations::pointIsInDefenseArea(field, trial, false)) {
                    auto fieldDiagonalLength = sqrt(fieldWidth * fieldWidth + fieldLength * fieldLength);

                    // Search closest bot to this point and get that distance
                    auto theirClosestBot = w.getRobotClosestToPoint(trial, rtt::world::Team::them);
                    auto theirClosestBotDistance{1.0};
                    if (theirClosestBot) {
                        theirClosestBotDistance = theirClosestBot.value()->getPos().dist(trial) / fieldDiagonalLength;
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