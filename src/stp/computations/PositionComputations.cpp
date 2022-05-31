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
    double spaceBetweenDefenseArea = 2 * radius;  // Because path planning is weird about being right next to a defense area

    Vector2 ballPos = FieldComputations::projectPointInField(field, world->getWorld().value().getBall()->get()->getPos());

    std::vector<Vector2> positions = {};

    Vector2 lineBorderIntersect;
    std::vector<Vector2> lineBorderIntersects = {};

    /// Get defense area border geometry
    std::vector<LineSegment> defenseAreaBorder = FieldComputations::getDefenseArea(field, true, spaceBetweenDefenseArea, 0).getBoundary();

    /// Find intersect of ball to goal on the border of the defense area
    LineSegment ball2GoalLine = LineSegment(ballPos, field.getOurGoalCenter());
    for (auto i : defenseAreaBorder) {
        for (const LineSegment line : defenseAreaBorder) {  // Vector is made to check if there is only 1 intersect
            if (line.doesIntersect(ball2GoalLine)) {
                auto intersect = line.intersects(ball2GoalLine);
                if (intersect.has_value() &&
                    // check if there is an intersect and that the intersect is not with the goal line
                    intersect->x - field.getOurGoalCenter().x > radius + control_constants::GO_TO_POS_ERROR_MARGIN)
                    lineBorderIntersects.push_back(intersect.value());
            }
        }
    }

    if (!lineBorderIntersects.empty()) {
        lineBorderIntersect = lineBorderIntersects.front();  // Always use the first (as there should only be one).
    }

    Vector2 projectedPosition = lineBorderIntersect;

    RTT_DEBUG("Position ", projectedPosition);

    LineSegment wallLine;

    bool wallIsSlanted = false;

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
        wallIsSlanted = true;
    }

    int i = 1;
    if (amountDefenders % 2 == 0) {
        int j = 1;
        while (positions.size() < static_cast<size_t>(amountDefenders)) {
            auto circle = Circle(projectedPosition, (i - 0.5) * spacingRobots + j * radius);
            std::vector<Vector2> intersects;
            if (wallIsSlanted) {
                intersects = circle.intersects(wallLine);
            } else {
                intersects = circle.intersectsCircleWithLineSegment(wallLine);
            }
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
            if (wallIsSlanted) {
                intersects = circle.intersects(wallLine);
            } else {
                intersects = circle.intersectsCircleWithLineSegment(wallLine);
            }
            for (auto intersect : intersects) {
                positions.push_back(intersect);
            }
            i = i + 1;
        }
    }

    // get the check where to put remaining defenders if the position array is not full
    // add a check for when the last position is outside of the field
    /// No intersects with defense area: The ball should be outside the field if this happens.
    /// Place robots in the same spot everytime when this happens, if no positions it segfaults.
    if (!FieldComputations::pointIsInField(field, ballPos)) {
        RTT_DEBUG("Waller in Class ifif");

        if (FieldComputations::pointIsInOurDefenseArea(field, world->getWorld()->getBall()->get()->getPos(), 0.5, 1) ||
            !FieldComputations::pointIsInField(field, world->getWorld()->getBall()->get()->getPos(), 0)) {
            double wallPosX = 0.4 * field.getFieldLength();
            double posX = field.getOurGoalCenter().x < 0 ? -wallPosX : wallPosX;
            positions.reserve(amountDefenders);
            for (int i = 0; i < amountDefenders; i++) {
                positions.emplace_back(Vector2{posX, field.getBottomLeftOurDefenceArea().y + i * control_constants::ROBOT_RADIUS * 3});
            }
        }
    }

    std::sort(std::begin(positions), std::end(positions), [](Vector2 a, Vector2 b) { return a.length() > b.length(); });
    RTT_DEBUG("POS SIZE ", positions.size())

    return positions;

}

}  // namespace rtt::ai::stp