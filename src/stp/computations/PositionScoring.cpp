//
// Created by alexander on 02-12-21.
//

#include "stp/computations/PositionScoring.h"

#include <optional>

#include "stp/computations/ComputationManager.h"
#include "stp/evaluations/position/BlockingEvaluation.h"
#include "stp/evaluations/position/GoalShotEvaluation.h"
#include "stp/evaluations/position/LineOfSightEvaluation.h"
#include "stp/evaluations/position/OpennessEvaluation.h"

namespace rtt::ai::stp {
gen::ScoredPosition PositionScoring::scorePosition(const Vector2 &position, gen::ScoreProfile &profile, const world::Field &field, const world::World *world, uint8_t bias) {
    gen::PositionScores &scores = ComputationManager::calculatedScores[position];
    uint8_t positionScore = getScoreOfPosition(profile, position, scores, field, world);
    if (bias) positionScore = (positionScore + bias > bias) ? positionScore + bias : std::numeric_limits<uint8_t>::max();  // stop overflow of uint8_t (254+2 = 1)
    return {position, positionScore};
}

uint8_t PositionScoring::getScoreOfPosition(gen::ScoreProfile &profile, Vector2 position, gen::PositionScores &scores, const rtt::world::Field &field,
                                            const rtt::world::World *world) {
    double scoreTotal = 0;
    double weightTotal = 0;
    if (profile.weightGoalShot > 0) {
        scoreTotal += scores.scoreGoalShot.value_or(determineGoalShotScore(position, field, world, scores)) * profile.weightGoalShot;
        weightTotal += profile.weightGoalShot;
    }
    if (profile.weightLineOfSight > 0) {
        scoreTotal += scores.scoreLineOfSight.value_or(determineLineOfSightScore(position, world, scores)) * profile.weightLineOfSight;
        weightTotal += profile.weightLineOfSight;
    }
    if (profile.weightOpen > 0) {
        scoreTotal += scores.scoreOpen.value_or(determineOpenScore(position, world, scores)) * profile.weightOpen;
        weightTotal += profile.weightOpen;
    }
    if (profile.weightBlocking) {
        scoreTotal += scores.scoreBlocking.value_or(determineBlockingScore(position, world, scores)) * profile.weightBlocking;
        weightTotal += profile.weightBlocking;
    }
    return static_cast<uint8_t>(scoreTotal / weightTotal);
}

double PositionScoring::determineOpenScore(Vector2 &point, const rtt::world::World *world, gen::PositionScores &scores) {
    std::vector<double> enemyDistances;
    auto &them = world->getWorld()->getThem();
    enemyDistances.reserve(them.size());
    for (auto &enemyRobot : them) {
        enemyDistances.push_back(point.dist(enemyRobot->getPos()));
    }
    return (scores.scoreOpen = stp::evaluation::OpennessEvaluation().metricCheck(enemyDistances)).value();
}

double PositionScoring::determineLineOfSightScore(Vector2 &point, const rtt::world::World *world, gen::PositionScores &scores) {
    Vector2 ballPos = world->getWorld().value()->getBall()->get()->getPos();
    double pointDistance = ballPos.dist(point);
    double pointAngle = (ballPos - point).angle();
    std::vector<double> enemyDistancesToBall;
    std::vector<double> enemyAnglesToBallvsPoint;
    auto &them = world->getWorld()->getThem();
    enemyDistancesToBall.reserve(them.size());
    enemyAnglesToBallvsPoint.reserve(them.size());
    for (auto &enemyRobot : them) {
        enemyDistancesToBall.push_back(ballPos.dist(enemyRobot->getPos()));
        enemyAnglesToBallvsPoint.push_back((ballPos - enemyRobot->getPos()).angle() - pointAngle);
    }
    return (scores.scoreLineOfSight = stp::evaluation::LineOfSightEvaluation().metricCheck(pointDistance, enemyDistancesToBall, enemyAnglesToBallvsPoint)).value();
}

double PositionScoring::determineGoalShotScore(Vector2 &point, const rtt::world::Field &field, const rtt::world::World *world, gen::PositionScores &scores) {
    double visibility = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, false, point, world->getWorld().value()) / 100;
    double goalDistance = FieldComputations::getDistanceToGoal(field, false, point);
    double trialToGoalAngle = fabs((field.getTheirGoalCenter() - point).angle());
    return (scores.scoreGoalShot = stp::evaluation::GoalShotEvaluation().metricCheck(visibility, goalDistance, trialToGoalAngle)).value();
}

double PositionScoring::determineBlockingScore(Vector2 &point, const rtt::world::World *world, gen::PositionScores &scores) {
    Vector2 ballPos = world->getWorld().value()->getBall()->get()->getPos();
    double pointDistance = ballPos.dist(point);
    double pointAngle = (point - ballPos).angle();
    std::vector<double> enemyDistances;
    std::vector<double> enemyAnglesToBallvsPoint;
    auto &them = world->getWorld()->getThem();
    enemyDistances.reserve(them.size());
    enemyAnglesToBallvsPoint.reserve(them.size());
    for (auto &enemyRobot : them) {
        enemyDistances.push_back(point.dist(enemyRobot->getPos()));
        enemyAnglesToBallvsPoint.push_back((enemyRobot->getPos() - ballPos).angle() - pointAngle);
    }
    return (scores.scoreBlocking = stp::evaluation::BlockingEvaluation().metricCheck(pointDistance, enemyDistances, enemyAnglesToBallvsPoint)).value();
}
}  // namespace rtt::ai::stp