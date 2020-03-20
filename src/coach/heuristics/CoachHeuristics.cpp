//
// Created by robzelluf on 3/21/19.
//

#include "coach/heuristics/CoachHeuristics.h"
#include <analysis/GameAnalyzer.h>
#include <include/roboteam_ai/world_new/World.hpp>
#include "control/ControlUtils.h"
#include "world/FieldComputations.h"

namespace rtt::ai::coach {

const double CoachHeuristics::CLOSE_TO_GOAL_WEIGHT = -0.1;
const double CoachHeuristics::SHOT_AT_GOAL_WEIGHT = -3.0;
const double CoachHeuristics::PASS_LINE_WEIGHT = -3.0;
const double CoachHeuristics::DISTANCE_TO_OPPONENTS_WEIGHT = -3.0;
const double CoachHeuristics::DISTANCE_TO_US_WEIGHT = -3.0;
const double CoachHeuristics::ANGLE_TO_GOAL_WEIGHT = -1.0;

const double CoachHeuristics::MAX_INTERCEPT_ANGLE = M_PI / 4.0;

/// Gives a higher score to positions closer to the oppontents goal
double CoachHeuristics::calculateCloseToGoalScore(const world::Field &field, const Vector2 &position) {
    double distanceFromGoal = (field.getTheirGoalCenter() - position).length();

    double score = exp(CLOSE_TO_GOAL_WEIGHT * distanceFromGoal);
    return score;
}

/// Gives a higher score if the line between the position and the goal is free.
double CoachHeuristics::calculateShotAtGoalScore(const world::Field &field, const Vector2 &position, world_new::view::WorldDataView world) {
    double viewAtGoal = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, false, position, world) / 100;
    return 1 - exp(SHOT_AT_GOAL_WEIGHT * viewAtGoal);
}

/// Gives a higher score if the distance between the ball and the positions if free (safe pass line)
double CoachHeuristics::calculatePassLineScore(const Vector2 &position, world_new::view::WorldDataView world) {
    double smallestAngle = MAX_INTERCEPT_ANGLE;
    smallestAngle = getClosestOpponentAngleToPassLine(position, world, smallestAngle);
    return 1 - exp(PASS_LINE_WEIGHT * smallestAngle);
}

double CoachHeuristics::getClosestOpponentAngleToPassLine(const Vector2 &position, world_new::view::WorldDataView world, double smallestAngle) {
    auto ball = world.getBall();
    for (const auto &robot : world.getThem()) {
        if (control::ControlUtils::isPointProjectedOnLineSegment(robot->getPos(), ball->get()->getPos(), position)) {
            double angle = abs((position - ball->get()->getPos()).toAngle() - (robot->getPos() - ball->get()->getPos()).toAngle());
            if (angle < smallestAngle) {
                smallestAngle = angle;
            }
        }
    }
    return smallestAngle;
}

/// Gives a higher score if the position is far away from enemy robots
double CoachHeuristics::calculateDistanceToOpponentsScore(const Vector2 &position) {
    world_new::view::RobotView closestRobot = world_new::World::instance()->getWorld()->getRobotClosestToPoint(position, world_new::Team::them);
    if (closestRobot && closestRobot->getId() != -1) {
        double distance = (position - closestRobot->getPos()).length();
        return 1 - exp(DISTANCE_TO_OPPONENTS_WEIGHT * distance);
    } else {
        return 1;
    }
}

double CoachHeuristics::calculateBehindBallScore(const Vector2 &position, world_new::view::WorldDataView world) {
    if (!world.getBall().has_value()) return 0.0;

    double xDistanceBehindBall = world.getBall()->get()->getPos().x - position.x;
    if (xDistanceBehindBall > 0) {
        return 0.0;
    } else {
        return 1.0;
    }
}

double CoachHeuristics::calculatePassDistanceToBallScore(const world::Field &field, const Vector2 &position, world_new::view::WorldDataView world) {
    auto ball = world.getBall()->get();
    double idealDistance = (field.getTheirGoalCenter() - ball->getPos()).length() * 0.5;
    double distanceFromBall = (position - ball->getPos()).length();

    if (distanceFromBall < Constants::MAX_PASS_DISTANCE()) {
        return -1;
    }

    return fmax(0.0, -pow(distanceFromBall / (0.5 * idealDistance), 2.0) + 2.0 * (distanceFromBall / (0.5 * idealDistance)));
}

double CoachHeuristics::calculatePositionDistanceToBallScore(const world::Field &field, const Vector2 &position, world_new::view::WorldDataView world) {
    auto ball = world.getBall()->get();
    double idealDistance = (field.getTheirGoalCenter() - ball->getPos()).length() * 0.75;
    double distanceFromBall = (position - ball->getPos()).length();
    return fmax(0.0, -pow(distanceFromBall / (0.5 * idealDistance), 2.0) + 2.0 * (distanceFromBall / (0.5 * idealDistance)));
}

double CoachHeuristics::calculateDistanceToClosestTeamMateScore(const Vector2 &position, int thisRobotID) {
    std::set<uint8_t> idVector;
    for (auto &robot : world_new::World::instance()->getWorld()->getUs()) {
        if (robot->getId() != thisRobotID) {
            idVector.insert(robot->getId());
        }
    }

    world_new::view::RobotView closestRobot = world_new::World::instance()->getWorld()->getRobotClosestToPoint(position, idVector, true);
    if (closestRobot && closestRobot->getId() != -1) {
        double distance = (position - closestRobot->getPos()).length();
        return 1.0 - exp(DISTANCE_TO_US_WEIGHT * distance);
    } else {
        return 1.0;
    }
}

double CoachHeuristics::calculateAngleToGoalScore(const world::Field &field, const Vector2 &position) {
    auto goalSides = FieldComputations::getGoalSides(field, false);
    Angle angle1 = (goalSides.start - position).toAngle();
    Angle angle2 = (goalSides.end - position).toAngle();

    Angle angleToGoal = abs(angle2 - angle1);

    return 1.0 - exp(ANGLE_TO_GOAL_WEIGHT * angleToGoal);
}

}  // namespace rtt::ai::coach
