//
// Created by mrlukasbos on 19-3-19.
//

/*
 *
 * This class contains functionality for retreiving some tactical positions in the game
 *
 */

#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/world/Field.h>
#include "PositionUtils.h"

namespace rtt {
namespace ai {
namespace control {

rtt::Vector2 PositionUtils::getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal) {
    const Vector2 &goal = (ourGoal ? world::field->get_our_goal_center() : world::field->get_their_goal_center());
    return getPositionBehindBallToPosition(distanceBehindBall, goal);
}

Vector2 PositionUtils::getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot,
        const unsigned int &robotID) {
    Vector2 robot;
    if (world::world->getRobotForId(robotID, ourRobot)) {
        robot = world::world->getRobotForId(robotID, ourRobot).get()->pos;
        return getPositionBehindBallToPosition(distanceBehindBall, robot);
    }
    return Vector2();
}

Vector2 PositionUtils::getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position) {
    auto ball = world::world->getBall();
    if (! ball) return {};
    Vector2 ballPos = ball->pos;
    return ballPos + (ballPos - position).stretchToLength(distanceBehindBall);
}

Vector2 PositionUtils::getPositionBehindPositionToPosition(
        double distanceBehindBall, const Vector2 &behindPosition, const Vector2 &toPosition) {
    return behindPosition + (behindPosition - toPosition).stretchToLength(distanceBehindBall);
}

bool PositionUtils::isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPosition, double angleMargin) {
    const Vector2 &goal = (ourGoal ? world::field->get_our_goal_center() : world::field->get_their_goal_center());
    return isRobotBehindBallToPosition(distanceBehindBall, goal, robotPosition, angleMargin);
}

bool PositionUtils::isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID,

        const Vector2 &robotPosition, double angleMargin) {
    Vector2 robot;
    if (world::world->getRobotForId(robotID, ourRobot)) {
        robot = world::world->getRobotForId(robotID, ourRobot).get()->pos;
        return isRobotBehindBallToPosition(distanceBehindBall, robot, robotPosition, angleMargin);
    }
    return false;
}

bool PositionUtils::isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position,
        const Vector2 &robotPosition, double angleMargin) {

    const Vector2 &ball = static_cast<Vector2>(world::world->getBall()->pos);
    Vector2 behindBallPosition = getPositionBehindBallToPosition(distanceBehindBall, position);
    Vector2 deltaBall = behindBallPosition - ball;

    Vector2 trianglePoint1 = ball;
    Vector2 trianglePoint2 = ball + deltaBall.rotate(M_PI*angleMargin).scale(2.0);
    Vector2 trianglePoint3 = ball + deltaBall.rotate(M_PI*- angleMargin).scale(2.0);

    bool inLargeTriangleOnPosition = control::ControlUtils::pointInTriangle(robotPosition, trianglePoint1,
            trianglePoint2, trianglePoint3);

    return inLargeTriangleOnPosition;
}


} // coach
} // ai
} // rtt