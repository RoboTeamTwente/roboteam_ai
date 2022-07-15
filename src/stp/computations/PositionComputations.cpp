//
// Created by maxl on 09-02-21.
//

#include "stp/computations/PositionComputations.h"

#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Tube.h>

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

    Vector2 ballPos = FieldComputations::projectPointInField(field, world->getWorld().value().getBall()->get()->position);

    std::vector<Vector2> positions = {};

    Vector2 projectedPosition;
    std::vector<Vector2> lineBorderIntersects = {};

    /// Get defense area border geometry
    std::vector<LineSegment> defenseAreaBorder = FieldComputations::getDefenseArea(field, true, spaceBetweenDefenseArea, 0).getBoundary();

    /// Find intersect of ball to goal on the border of the defense area
    LineSegment ball2GoalLine = LineSegment(ballPos, field.getOurGoalCenter());

    lineBorderIntersects = FieldComputations::getDefenseArea(field, true, spaceBetweenDefenseArea, 0).intersections(ball2GoalLine);

    if (!lineBorderIntersects.empty()) {
        std::sort(std::begin(lineBorderIntersects), std::end(lineBorderIntersects), [](Vector2 a, Vector2 b) { return a.x > b.x; });
        projectedPosition = lineBorderIntersects.front();  // Always use the first one
    } else {
        projectedPosition = Vector2{field.getOurGoalCenter().x, field.getBottomLeftOurDefenceArea().y};
    }

    LineSegment wallLine;

    // TODO: The wall line used to be as long as the line of the defense area it belonged to.
    // This meant that later, when we intersect it with a circle to decide the positions of the robots,
    // there is a possibility that the circle becomes bigger than the linesegment, resulting in an infinite loop.
    // A quick fix for this is extending the line segments, but in my eyes, it is a beun fix, because there is
    // still no guarantee it is big enough
    if (ballPos.x < field.getLeftPenaltyLineBottom().x) {
        // case when the projected position is below penalty line
        if (ballPos.y < 0) {
            wallLine = LineSegment(Vector2{FieldComputations::getDefenseArea(field, true, 0, 0)[0].x, FieldComputations::getDefenseArea(field, true, 0, 0)[0].y},
                                   Vector2{FieldComputations::getDefenseArea(field, false, 0, 0)[0].x, FieldComputations::getDefenseArea(field, false, 0, 0)[0].y});
//            wallLine = LineSegment(field.getBottomLeftOurDefenceArea(), field.getBottomRightTheirDefenceArea());
            wallLine.move({0, -spaceBetweenDefenseArea});
        } else {
            wallLine = LineSegment(Vector2{FieldComputations::getDefenseArea(field, true, 0, 0)[3].x, FieldComputations::getDefenseArea(field, true, 0, 0)[3].y},
                                   Vector2{FieldComputations::getDefenseArea(field, false, 0, 0)[3].x, FieldComputations::getDefenseArea(field, false, 0, 0)[3].y});
//            wallLine = LineSegment(field.getTopLeftOurDefenceArea(), field.getTopRightTheirDefenceArea());
            wallLine.move({0, spaceBetweenDefenseArea});
        }
        // case when it is above the penalty line no further away than side lines of the defense area
    } else if (ballPos.y < field.getLeftPenaltyLineTop().y && ballPos.y > field.getLeftPenaltyLineBottom().y) {
        double lineX = field.getLeftPenaltyX() + spaceBetweenDefenseArea;
        double lineYTop = field.getTopmostY();
        double lineYBottom = field.getBottommostY();
        wallLine = LineSegment({lineX, lineYBottom}, {lineX, lineYTop});
    } else {
        // We put the wall line perpendicular to the ball-goal line
        wallLine = LineSegment(ballPos, field.getOurGoalCenter());
        wallLine.rotate(M_PI / 2, projectedPosition);

        // And resize it to make sure enough robots can fit on it
        double newLength = 2 * std::max(field.getFieldWidth(), field.getFieldLength());
        wallLine.resize(newLength);

        // But limit this resizing to the edges of the field (we dont want to place robots outside of the field
        auto oneHalf = LineSegment(wallLine.center(), wallLine.start);
        auto intersectionOne = FieldComputations::lineIntersectionWithField(field, oneHalf.start, oneHalf.end, 0.0);
        if (intersectionOne.has_value()) {
            // Then limit this side of the wall line until this intersection
            wallLine.start = intersectionOne.value();
        }

        auto otherHalf = LineSegment(wallLine.center(), wallLine.end);
        auto intersectionOther = FieldComputations::lineIntersectionWithField(field, otherHalf.start, otherHalf.end, 0.0);
        if (intersectionOther.has_value()) {
            // Then limit this side of the wall line until this intersection
            wallLine.end = intersectionOther.value();
        }
    }

    int i = 1;
    if (amountDefenders % 2 == 0) {
        int j = 1;
        while (positions.size() < static_cast<size_t>(amountDefenders)) {
            auto circle = Circle(projectedPosition, (i - 0.5) * spacingRobots + j * radius);
            std::vector<Vector2> intersects;
            intersects = circle.intersects(wallLine);
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
            intersects = circle.intersects(wallLine);
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
Vector2 PositionComputations::calculateAvoidBallPosition(Vector2 targetPosition, Vector2 ballPosition, const world::Field &field) {
    auto currentGameState = GameStateManager::getCurrentGameState().getStrategyName();

    std::unique_ptr<Shape> avoidShape;

    // During ball placement, we need to avoid the area between the ball and the target position by a certain margin
    if (currentGameState == "ball_placement_us" || currentGameState == "ball_placement_them") {
        avoidShape = std::make_unique<Tube>(Tube(ballPosition, GameStateManager::getRefereeDesignatedPosition(), control_constants::AVOID_BALL_DISTANCE));
    } else {
        // During stop gamestate, we need to avoid the area directly around the ball.
        avoidShape = std::make_unique<Circle>(Circle(ballPosition, control_constants::AVOID_BALL_DISTANCE));
    }

    if (avoidShape->contains(targetPosition)) {
        auto projectedPos = avoidShape->project(targetPosition);
        if (FieldComputations::pointIsValidPosition(field, projectedPos))
            targetPosition = projectedPos;
        else {
            targetPosition = calculatePositionOutsideOfShape(ballPosition, field, avoidShape);
        }
    }
    return targetPosition;
}

Vector2 PositionComputations::calculatePositionOutsideOfShape(Vector2 ballPos, const rtt::world::Field &field, const std::unique_ptr<Shape> &avoidShape) {
    Vector2 newTarget = ballPos;  // The new position to go to
    bool pointFound = false;
    for (int distanceSteps = 0; distanceSteps < 5; ++distanceSteps) {
        // Use a larger grid each iteration in case no valid point is found
        auto distance = 3 * control_constants::AVOID_BALL_DISTANCE + distanceSteps * control_constants::AVOID_BALL_DISTANCE / 2.0;
        auto possiblePoints = Grid(ballPos.x - distance / 2.0, ballPos.y - distance / 2.0, distance, distance, 3, 3).getPoints();
        double dist = 1e3;
        for (auto &pointVector : possiblePoints) {
            for (auto &point : pointVector) {
                if (FieldComputations::pointIsValidPosition(field, point) && !avoidShape->contains(point)) {
                    if (ballPos.dist(point) < dist) {
                        dist = ballPos.dist(point);
                        newTarget = point;
                        pointFound = true;
                    }
                }
            }
        }
        if (pointFound) break;  // As soon as a valid point is found, don't look at more points further away
    }
    if (newTarget == ballPos) RTT_WARNING("Could not find good position to avoid ball");
    return newTarget;
}

Vector2 PositionComputations::getBallBlockPosition(const world::Field &field, const world::World *world) {
    if (!world->getWorld()->getBall()) return {field.getLeftPenaltyPoint()};  // If there is no ball, return a default value

    constexpr double distFromDefenceArea = 1.0;

    // If the ball is within this distFromDefence area, go to the ball
    if (FieldComputations::getDefenseArea(field, true, distFromDefenceArea, 0).contains(world->getWorld()->getBall()->get()->position)){
        return world->getWorld()->getBall()->get()->position;
    }
    // If the ball is moving towards our defense area, stand on its trajectory
    auto ball = world->getWorld()->getBall()->get();
    if (ball->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT) {
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.getFieldLength()));
        auto interceptionPoint =
            FieldComputations::lineIntersectionWithDefenseArea(field, true, ballTrajectory.start, ballTrajectory.end, distFromDefenceArea, true);
        if (interceptionPoint) return *interceptionPoint;
    }

    // If there is an enemy close to the ball and it is looking towards our goal, stand on the line that the enemy is looking in
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(rtt::world::Team::them);
    if (enemyClosestToBall && enemyClosestToBall->get()->getDistanceToBall() < control_constants::ENEMY_CLOSE_TO_BALL_DISTANCE) {
        auto start = enemyClosestToBall->get()->getPos();
        auto robotAngle = enemyClosestToBall->get()->getAngle();
        auto end = start + robotAngle.toVector2().stretchToLength(field.getFieldLength());
        //auto intersection = LineSegment(field.getOurBottomGoalSide() - Vector2(0, 0.20), field.getOurTopGoalSide() + Vector2(0, 0.20)).intersects({start, end});
        auto intersection = FieldComputations::lineIntersectionWithDefenseArea(field, true, start, end, distFromDefenceArea, true);
        if (intersection != nullptr) {
            return FieldComputations::projectPointToValidPositionOnLine(field, *intersection, start, end, AvoidObjects(), 0.0,
                                                                        distFromDefenceArea, distFromDefenceArea);
        }
    }

    // If there is no enemy about to shoot and the ball is not moving towards the goal, simply stand in between the ball and our goal center
    auto ballToGoalIntersection =
        FieldComputations::lineIntersectionWithDefenseArea(field, true, ball->position, field.getOurGoalCenter(), distFromDefenceArea, true);
    if (ballToGoalIntersection) return *ballToGoalIntersection;

    // If there is no ball to goal intersection (this essentially means the ball is in our defense area), project that the ball position to a valid point
    return {FieldComputations::projectPointToValidPosition(field, ball->position, AvoidObjects{}, 0.0, distFromDefenceArea, distFromDefenceArea)};
}

}  // namespace rtt::ai::stp