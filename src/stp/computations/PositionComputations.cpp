//
// Created by maxl on 09-02-21.
//

#include "stp/computations/PositionComputations.h"
#include "stp/StpInfo.h"

#include <roboteam_utils/Grid.h>

#include "world/Field.h"
#include "world/World.hpp"

#include "stp/evaluations/position/OpennessEvaluation.h"
#include "stp/evaluations/position/LineOfSightEvaluation.h"
#include "stp/evaluations/position/GoalShotEvaluation.h"
#include "stp/evaluations/position/BlockingEvaluation.h"

namespace rtt::ai::stp {

    gen::ScoredPosition
    PositionComputations::getPosition(std::optional<rtt::Vector2> currentPosition, const Grid &searchGrid,
                                      gen::ScoreProfile profile, const world::Field &field, world::World *world) {
        gen::ScoredPosition bestPosition;
        (currentPosition.has_value()) ? bestPosition = scorePosition(currentPosition.value(), profile, field, world, 2)
                                      : bestPosition = {{0, 0}, 0};
        for (const auto &nestedPoints : searchGrid.getPoints()) {
            for (const Vector2 &position : nestedPoints) {
                if (!FieldComputations::pointIsValidPosition(field, position)) continue;
                gen::ScoredPosition consideredPosition = scorePosition(position, profile, field, world);
                if (consideredPosition.score > bestPosition.score) bestPosition = consideredPosition;
            }
        }
        return bestPosition;
    }

    gen::ScoredPosition
    PositionComputations::scorePosition(const Vector2 &position, gen::ScoreProfile &profile, const world::Field &field,
                                        const world::World *world, uint8_t bias) {
        gen::PositionScores &scores = calculatedScores[position];
        uint8_t positionScore = getScoreOfPosition(profile, position, scores, field, world);
        if (bias)
            positionScore = (positionScore + bias > bias) ? positionScore + bias
                                                          : std::numeric_limits<uint8_t>::max(); //stop overflow of uint8_t (254+2 = 1)
        return {position, positionScore};
    }

    uint8_t
    PositionComputations::getScoreOfPosition(gen::ScoreProfile &profile, Vector2 position, gen::PositionScores &scores,
                                             const rtt::world::Field &field, const rtt::world::World *world) {
        double scoreTotal = 0;
        double weightTotal = 0;
        if (profile.weightGoalShot > 0) {
            scoreTotal += scores.scoreGoalShot.value_or(determineGoalShotScore(position, field, world, scores)) *
                          profile.weightGoalShot;
            weightTotal += profile.weightGoalShot;
        }
        if (profile.weightLineOfSight > 0) {
            scoreTotal += scores.scoreLineOfSight.value_or(determineLineOfSightScore(position, world, scores)) *
                          profile.weightLineOfSight;
            weightTotal += profile.weightLineOfSight;
        }
        if (profile.weightOpen > 0) {
            scoreTotal += scores.scoreOpen.value_or(determineOpenScore(position, world, scores)) * profile.weightOpen;
            weightTotal += profile.weightOpen;
        }
        if (profile.weightBlocking) {
            scoreTotal += scores.scoreBlocking.value_or(determineBlockingScore(position, world, scores)) *
                          profile.weightBlocking;
            weightTotal += profile.weightBlocking;
        }
        return static_cast<uint8_t>(scoreTotal / weightTotal);
    }

    double
    PositionComputations::determineOpenScore(Vector2 &point, const rtt::world::World *world, gen::PositionScores &scores) {
        std::vector<double> enemyDistances;
        auto& them = world->getWorld()->getThem();
        enemyDistances.reserve(them.size());
        for (auto &enemyRobot : them) {
            enemyDistances.push_back(point.dist(enemyRobot->getPos()));
        }
        return (scores.scoreOpen = stp::evaluation::OpennessEvaluation().metricCheck(enemyDistances)).value();
    }

    double PositionComputations::determineLineOfSightScore(Vector2 &point, const rtt::world::World *world,
                                                           gen::PositionScores &scores) {
        Vector2 ballPos = world->getWorld().value()->getBall()->get()->getPos();
        double pointDistance = ballPos.dist(point);
        double pointAngle = (ballPos - point).angle();
        std::vector<double> enemyDistancesToBall;
        std::vector<double> enemyAnglesToBallvsPoint;
        auto& them = world->getWorld()->getThem();
        enemyDistancesToBall.reserve(them.size());
        enemyAnglesToBallvsPoint.reserve(them.size());
        for (auto &enemyRobot : them) {
            enemyDistancesToBall.push_back(ballPos.dist(enemyRobot->getPos()));
            enemyAnglesToBallvsPoint.push_back((ballPos - enemyRobot->getPos()).angle() - pointAngle);
        }
        return (scores.scoreLineOfSight = stp::evaluation::LineOfSightEvaluation().metricCheck(pointDistance,
                                                                                               enemyDistancesToBall,
                                                                                               enemyAnglesToBallvsPoint)).value();
    }

    double PositionComputations::determineGoalShotScore(Vector2 &point, const rtt::world::Field &field,
                                                        const rtt::world::World *world, gen::PositionScores &scores) {
        double visibility = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, false, point, world) / 100;
        double goalDistance = FieldComputations::getDistanceToGoal(field, false, point);
        double trialToGoalAngle = fabs((field.getTheirGoalCenter() - point).angle());
        return (scores.scoreGoalShot = stp::evaluation::GoalShotEvaluation().metricCheck(visibility, goalDistance,
                                                                                         trialToGoalAngle)).value();
    }

    double PositionComputations::determineBlockingScore(Vector2 &point, const rtt::world::World *world,
                                                        gen::PositionScores &scores) {
        Vector2 ballPos = world->getWorld().value()->getBall()->get()->getPos();
        double pointDistance = ballPos.dist(point);
        double pointAngle = (point - ballPos).angle();
        std::vector<double> enemyDistances;
        std::vector<double> enemyAnglesToBallvsPoint;
        auto& them = world->getWorld()->getThem();
        enemyDistances.reserve(them.size());
        enemyAnglesToBallvsPoint.reserve(them.size());
        for (auto &enemyRobot : them) {
            enemyDistances.push_back(point.dist(enemyRobot->getPos()));
            enemyAnglesToBallvsPoint.push_back((enemyRobot->getPos() - ballPos).angle() - pointAngle);
        }
        return (scores.scoreBlocking = stp::evaluation::BlockingEvaluation().metricCheck(pointDistance, enemyDistances,
                                                                                         enemyAnglesToBallvsPoint)).value();
    }

    Vector2 PositionComputations::getWallPosition(int index, int amountDefenders, const rtt::world::Field &field,
                                                  rtt::world::World *world) {
        if (calculatedWallPositions.empty()) {
            calculatedWallPositions = determineWallPositions(field, world, amountDefenders);
        }
        return calculatedWallPositions[index];
    }

    std::vector<Vector2>
    PositionComputations::determineWallPositions(const rtt::world::Field &field, const rtt::world::World *world,
                                                 int amountDefenders) {
        Vector2 ballPos = FieldComputations::placePointInField(field, world->getWorld().value().getBall()->get()->getPos());
        double radius = control_constants::ROBOT_RADIUS;
        double spacingRobots = radius * 2;
        double spaceBetweenDefenseArea = field.getFieldLength() / 30; //Because path planning is weird about being right next to a defense area

        std::vector<Vector2> positions = {};
        Vector2 lineBorderIntersect;
        std::vector<Vector2> lineBorderIntersects = {};

        /// Get defense area border geometry
        std::vector<LineSegment> defenseAreaBorder = FieldComputations::getDefenseArea(field, true,
                                                                                       radius +
                                                                                       control_constants::GO_TO_POS_ERROR_MARGIN,
                                                                                       0).getBoundary();

        /// Find intersect of ball to goal on the border of the defense area
        LineSegment ball2GoalLine = LineSegment(ballPos, field.getOurGoalCenter());
        for (auto &i : defenseAreaBorder) {
            for (const LineSegment &line : defenseAreaBorder) { // Vector is made to check if there is only 1 intersect
                if (line.doesIntersect(ball2GoalLine)) {
                    auto intersect = line.intersects(ball2GoalLine);
                    if (intersect.has_value() &&
                        // check if there is an intersect and that the intersect is not with the goal line
                        intersect->x - field.getOurGoalCenter().x > radius +
                                                                    control_constants::GO_TO_POS_ERROR_MARGIN)
                        lineBorderIntersects.push_back(intersect.value());
                }
            }
        }

        if (!lineBorderIntersects.empty()) {
            lineBorderIntersect = lineBorderIntersects.front();  // Always use the first (as there should only be one).
        }

        /// Intersect found with defense area:
        /// Place robots around the intersect in front of defense area
        int j = 1;
        double base = 0.5; //Offset if there are even defenders
        if ((amountDefenders) % 2) { //If odd, place 1 at the interest
            base = 0.0;
            positions.push_back(lineBorderIntersect);
        }
        while (positions.size() < amountDefenders) {
            auto circle = Circle(lineBorderIntersect, (base + j++) * (spacingRobots));
            for (const LineSegment &line : defenseAreaBorder) {
                auto intersects = circle.intersectsCircleWithLineSegment(circle, line);
                for (auto intersect : intersects) {
                    double spaceBetweenDefenseAreas = intersect.x < 0 ? spaceBetweenDefenseArea : -spaceBetweenDefenseArea; //Because path planning is weird about being right next to a defense area
                    positions.push_back(intersect + Vector2{spaceBetweenDefenseAreas, 0});
                }
            }
        }

        /// No intersects with defense area: The ball should be outside the field if this happens.
        /// Place robots in the same spot everytime when this happens, if no positions it segfaults.
        if (lineBorderIntersects.empty()) {
            if (FieldComputations::pointIsInDefenseArea(field, world->getWorld()->getBall()->get()->getPos(), true, 0.5,
                                                        1) ||
                !FieldComputations::pointIsInField(field, world->getWorld()->getBall()->get()->getPos(), 0)) {
                double wallPosX = 0.4*field.getFieldLength();
                double posX = field.getOurGoalCenter().x < 0 ? -wallPosX : wallPosX;
                positions.reserve(amountDefenders);
                for (int i = 0; i < amountDefenders; i++) {
                    positions.emplace_back(
                        Vector2{posX, field.getBottomLeftOurDefenceArea().y +
                                      i * control_constants::ROBOT_RADIUS * 3});
                }
            }
        }

        std::sort(std::begin(positions),std::end(positions),[](Vector2 a, Vector2 b) {return a.length() > b.length(); });
        return positions;
    }
} //namespace computations