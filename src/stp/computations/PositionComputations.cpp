//
// Created by maxl on 09-02-21.
//

#include "stp/computations/PositionComputations.h"

#include <roboteam_utils/Grid.h>

#include "stp/computations/ComputationManager.h"
#include "stp/computations/PositionScoring.h"
#include "world/Field.h"
#include "world/World.hpp"

namespace rtt::ai::stp {

gen::ScoredPosition PositionComputations::getPosition(std::optional<rtt::Vector2> currentPosition, const Grid &searchGrid, gen::ScoreProfile profile, const world::Field &field,
                                                      const world::World *world) {
    gen::ScoredPosition bestPosition;
    (currentPosition.has_value()) ? bestPosition = PositionScoring::scorePosition(currentPosition.value(), profile, field, world, 2) : bestPosition = {{0, 0}, 0};
    for (const auto &nestedPoints : searchGrid.getPoints()) {
        for (const Vector2 &position : nestedPoints) {
            if (!FieldComputations::pointIsValidPosition(field, position)) continue;
            gen::ScoredPosition consideredPosition = PositionScoring::scorePosition(position, profile, field, world);
            if (consideredPosition.score > bestPosition.score) bestPosition = consideredPosition;
        }
    }
    return bestPosition;
}

Vector2 PositionComputations::getWallPosition(int index, int amountDefenders, const rtt::world::Field &field, rtt::world::World *world) {
    if (ComputationManager::calculatedWallPositions.empty()) {
        ComputationManager::calculatedWallPositions = determineWallPositions(field, world, amountDefenders);
    }

    return ComputationManager::calculatedWallPositions[index];
}

std::vector<Vector2> PositionComputations::determineWallPositions(const rtt::world::Field &field, const rtt::world::World *world, int amountDefenders) {
    if (amountDefenders <= 0) return {};  // we need at least 1 defender to be able to compute a wall

    double radius = control_constants::ROBOT_RADIUS;
    double spacingRobots = radius * 0.5;
    double spaceBetweenDefenseArea = 2 * radius;

    Vector2 ballPos = FieldComputations::projectPointInField(field, world->getWorld().value().getBall()->get()->getPos());

    std::vector<Vector2> positions = {};

    Vector2 projectedPosition;
    std::vector<Vector2> lineBorderIntersects = {};

    /// Get defense area border geometry
    std::vector<LineSegment> defenseAreaBorder = FieldComputations::getDefenseArea(field, true, spaceBetweenDefenseArea, 0).getBoundary();

    /// Find intersect of ball to goal on the border of the defense area
    LineSegment ball2GoalLine = LineSegment(ballPos, field.getOurGoalCenter());
    for (auto i : defenseAreaBorder) {
        for (const LineSegment line : defenseAreaBorder) {  // Vector is made to check if there is only 1 intersect
            if (line.doesIntersect(ball2GoalLine)) {
                auto intersect = line.intersects(ball2GoalLine);
                if (intersect.has_value()) {
                    lineBorderIntersects.push_back(intersect.value());
                }
            }
        }
    }

    if (!lineBorderIntersects.empty()) {
        std::sort(std::begin(lineBorderIntersects), std::end(lineBorderIntersects), [](Vector2 a, Vector2 b) { return a.x > b.x; });
        projectedPosition = lineBorderIntersects.front();  // Always use the first one
    } else {
        projectedPosition = Vector2{field.getOurGoalCenter().x, field.getBottomLeftOurDefenceArea().y};
    }

    LineSegment wallLine;

    if (ballPos.x < field.getLeftPenaltyLineBottom().x) {
        // case when the projected position is below penalty line
        if (ballPos.y < 0) {
            wallLine = LineSegment(field.getBottomLeftOurDefenceArea(), field.getLeftPenaltyLineBottom());
            wallLine.move({0, -spaceBetweenDefenseArea});
        } else {
            wallLine = LineSegment(field.getTopLeftOurDefenceArea(), field.getLeftPenaltyLineTop());
            wallLine.move({0, spaceBetweenDefenseArea});
        }
        // case when it is above the penalty line no further away than side lines of the defense area
    } else if (ballPos.y < field.getLeftPenaltyLineTop().y && ballPos.y > field.getLeftPenaltyLineBottom().y) {
        wallLine = LineSegment(field.getLeftPenaltyLineBottom(), field.getLeftPenaltyLineTop());
        wallLine.move({spaceBetweenDefenseArea, 0});
    } else {
        wallLine = LineSegment(ballPos, field.getOurGoalCenter());
        wallLine.rotate(M_PI / 2, projectedPosition);
    }

    int i = 1;
    if (amountDefenders % 2 == 0) {
        int j = 1;
        while (positions.size() < static_cast<size_t>(amountDefenders)) {
            auto circle = Circle(projectedPosition, (i - 0.5) * spacingRobots + j * radius);
            std::vector<Vector2> intersects;
            intersects = circle.intersectsWithLineSegment(wallLine);
            for (auto intersect : intersects) {
                positions.push_back(intersect);
            }
            j = j + 2;
            i = i + 1;
        }
    } else {
        positions.push_back(projectedPosition);
        while (positions.size() < static_cast<size_t>(amountDefenders)) {
            auto circle = Circle(projectedPosition, i * 2 * radius + spacingRobots * i);
            std::vector<Vector2> intersects;
            intersects = circle.intersectsWithLineSegment(wallLine);
            for (auto intersect : intersects) {
                positions.push_back(intersect);
            }
            i = i + 1;
        }
    }

    // For the robots not to change the position withing the wall
    if (ballPos.x < field.getLeftPenaltyLineBottom().x) {
        if (ballPos.y < 0) {
            std::sort(std::begin(positions), std::end(positions), [](Vector2 a, Vector2 b) { return a.x < b.x; });
        } else {
            std::sort(std::begin(positions), std::end(positions), [](Vector2 a, Vector2 b) { return a.x > b.x; });
        }
    } else {
        std::sort(std::begin(positions), std::end(positions), [](Vector2 a, Vector2 b) { return a.y < b.y; });
    }

    return positions;
}

}  // namespace rtt::ai::stp