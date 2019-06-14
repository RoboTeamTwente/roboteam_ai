//
// Created by robzelluf on 3/21/19.
//

#include <roboteam_ai/src/analysis/GameAnalyzer.h>
#include "CoachHeuristics.h"
#include <cmath>

namespace rtt {
namespace ai {
namespace coach {

const double CoachHeuristics::CLOSE_TO_GOAL_WEIGHT = - 0.1;
const double CoachHeuristics::SHOT_AT_GOAL_WEIGHT = - 3.0;
const double CoachHeuristics::PASS_LINE_WEIGHT = - 3.0;
const double CoachHeuristics::DISTANCE_TO_OPPONENTS_WEIGHT = - 3.0;
const double CoachHeuristics::DISTANCE_TO_US_WEIGHT = - 3.0;
const double CoachHeuristics::ANGLE_TO_GOAL_WEIGHT = -1.0;

const double CoachHeuristics::MAX_INTERCEPT_ANGLE = M_PI/4.0;

/// Gives a higher score to positions closer to the oppontents goal
double CoachHeuristics::calculateCloseToGoalScore(const Vector2 &position) {
    double distanceFromGoal = (world::field->get_their_goal_center() - position).length();

    double score = exp(CLOSE_TO_GOAL_WEIGHT*distanceFromGoal);
    return score;
}

/// Gives a higher score if the line between the position and the goal is free.
double CoachHeuristics::calculateShotAtGoalScore(const Vector2 &position, const WorldData &world) {
    WorldData copy = WorldData({}, world.them, world.ball, world.time);
    double viewAtGoal = world::field->getPercentageOfGoalVisibleFromPoint(false, position, copy)/100;
    return 1 - exp(SHOT_AT_GOAL_WEIGHT*viewAtGoal);
}

/// Gives a higher score if the distance between the ball and the positions if free (safe pass line)
double CoachHeuristics::calculatePassLineScore(const Vector2 &position, const WorldData &world) {
    double smallestAngle = MAX_INTERCEPT_ANGLE;
    smallestAngle = getClosestOpponentAngleToPassLine(position, world, smallestAngle);
    return 1 - exp(PASS_LINE_WEIGHT*smallestAngle);
}

double CoachHeuristics::getClosestOpponentAngleToPassLine(const Vector2 &position, const WorldData &world,
        double smallestAngle) {

    auto ball = world.ball;
    for (const auto &robot : world.them) {
        if (control::ControlUtils::isPointProjectedOnLineSegment(robot->pos, ball->pos, position)) {
            double angle = abs((position - ball->pos).toAngle() - (robot->pos - ball->pos).toAngle());
            if (angle < smallestAngle) {
                smallestAngle = angle;
            }
        }
    }
    return smallestAngle;
}

/// Gives a higher score if the position is far away from enemy robots
double CoachHeuristics::calculateDistanceToOpponentsScore(const Vector2 &position) {
    RobotPtr closestRobot = world::world->getRobotClosestToPoint(position, THEIR_ROBOTS);
    if (closestRobot && closestRobot->id != - 1) {
        double distance = (position - closestRobot->pos).length();
        return 1 - exp(DISTANCE_TO_OPPONENTS_WEIGHT*distance);
    }
    else {
        return 1;
    }
}

double CoachHeuristics::calculateBehindBallScore(const Vector2 &position, const CoachHeuristics::WorldData &world) {
    if (!world.ball) return 0.0;

    double xDistanceBehindBall = world.ball->pos.x - position.x;
    if (xDistanceBehindBall > 0) {
        return 0.0;
    }
    else {
        return 1.0;
    }
}

double CoachHeuristics::calculateDistanceToBallScore(const Vector2 &position, const CoachHeuristics::WorldData &world) {
    auto ball = world.ball;
    double idealDistance = (world::field->get_their_goal_center() - ball->pos).length()*0.75;
    double distanceFromBall = (position - ball->pos).length();
    return fmax(0.0, - pow(distanceFromBall/(0.5*idealDistance), 2.0) + 2.0*(distanceFromBall/(0.5*idealDistance)));
}

double CoachHeuristics::calculateDistanceToClosestTeamMateScore(const Vector2 &position, int thisRobotID) {
    std::vector<int> idVector;
    for(auto &robot : world::world->getUs()) {
        if(robot->id != thisRobotID) {
            idVector.emplace_back(robot->id);
        }
    }

    RobotPtr closestRobot = world::world->getRobotClosestToPoint(position, idVector, true);
    if (closestRobot && closestRobot->id != - 1) {
        double distance = (position - closestRobot->pos).length();
        return 1.0 - exp(DISTANCE_TO_US_WEIGHT*distance);
    }
    else {
        return 1.0;
    }
}

double CoachHeuristics::calculateAngleToGoalScore(const Vector2 &position) {
    auto goalSides = world::field->getGoalSides(false);
    Angle angle1 = (goalSides.first - position).toAngle();
    Angle angle2 = (goalSides.second - position).toAngle();

    Angle angleToGoal = abs(angle2 - angle1);

    return 1.0 - exp(ANGLE_TO_GOAL_WEIGHT * angleToGoal);

}

}
}
}
