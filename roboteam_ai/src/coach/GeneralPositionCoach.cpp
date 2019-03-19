//
// Created by mrlukasbos on 19-3-19.
//

#include "GeneralPositionCoach.h"

Vector2 Coach::getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal) {
    const Vector2 &goal = (ourGoal ? Field::get_our_goal_center : Field::get_their_goal_center)();
    return getPositionBehindBallToPosition(distanceBehindBall, goal);
}

Vector2 Coach::getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID) {
    Vector2 robot;
    if (World::getRobotForId(robotID, ourRobot))
        robot = World::getRobotForId(robotID, ourRobot).get()->pos;
    else
        return Vector2();
    return getPositionBehindBallToPosition(distanceBehindBall, robot);
}

Vector2 Coach::getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position) {
    Vector2 ball = World::getBall()->pos;
    return ball + (ball - position).stretchToLength(distanceBehindBall);
}

bool Coach::isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPosition) {
    const Vector2 &goal = (ourGoal ? Field::get_our_goal_center : Field::get_their_goal_center)();
    return isRobotBehindBallToPosition(distanceBehindBall, goal, robotPosition);
}

bool Coach::isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID,
        const Vector2 &robotPosition) {
    Vector2 robot;
    if (World::getRobotForId(robotID, ourRobot))
        robot = World::getRobotForId(robotID, ourRobot).get()->pos;
    else
        return false;
    return isRobotBehindBallToPosition(distanceBehindBall, robot, robotPosition);
}

bool Coach::isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position,
        const Vector2 &robotPosition) {
    const Vector2 &ball = static_cast<Vector2>(World::getBall()->pos);
    Vector2 behindBallPosition = getPositionBehindBallToPosition(distanceBehindBall, position);
    Vector2 deltaBall = behindBallPosition - ball;

    double angleMargin = 0.12;

    return (control::ControlUtils::pointInTriangle(robotPosition, ball, ball + (deltaBall).rotate(M_PI*angleMargin).scale(2.0),
            ball + (deltaBall).rotate(M_PI*- angleMargin).scale(2.0)));
}
