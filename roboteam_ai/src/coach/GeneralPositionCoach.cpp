//
// Created by mrlukasbos on 19-3-19.
//

/*
 *
 * This class contains functionality for retreiving some tactical positions in the game
 *
 */

#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/utilities/Field.h>
#include "GeneralPositionCoach.h"

namespace rtt {
namespace ai {
namespace coach {

// create the global object
GeneralPositionCoach g_generalPositionCoach;

rtt::Vector2 GeneralPositionCoach::getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal) {
    const Vector2 &goal = (ourGoal ? Field::get_our_goal_center : Field::get_their_goal_center)();
    return getPositionBehindBallToPosition(distanceBehindBall, goal);
}

Vector2 GeneralPositionCoach::getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID) {
    Vector2 robot;
    if (World::getRobotForId(robotID, ourRobot)) {
        robot = World::getRobotForId(robotID, ourRobot).get()->pos;
        return getPositionBehindBallToPosition(distanceBehindBall, robot);
    }
    return Vector2();
}

Vector2 GeneralPositionCoach::getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position) {
    Vector2 ball = World::getBall()->pos;
    return ball + (ball - position).stretchToLength(distanceBehindBall);
}

Vector2
GeneralPositionCoach::getPositionBehindPositionToPosition(double distanceBehindBall, const Vector2 &behindPosition,
                                                          const Vector2 &toPosition) {
    return behindPosition + (behindPosition - toPosition).stretchToLength(distanceBehindBall);
}

bool GeneralPositionCoach::isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPosition, double angleMargin) {
    const Vector2 &goal = (ourGoal ? Field::get_our_goal_center : Field::get_their_goal_center)();
    return isRobotBehindBallToPosition(distanceBehindBall, goal, robotPosition, angleMargin);
}

bool GeneralPositionCoach::isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID,
        const Vector2 &robotPosition, double angleMargin) {
    Vector2 robot;
    if (World::getRobotForId(robotID, ourRobot)) {
        robot = World::getRobotForId(robotID, ourRobot).get()->pos;
        return isRobotBehindBallToPosition(distanceBehindBall, robot, robotPosition);
    }
    return false;
}

// get ball position
// get aposition behind the ball aimed at a position; e.g. the goal
// the delta with ball is the difference between the found position and the ball

//

bool GeneralPositionCoach::isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position,
        const Vector2 &robotPosition, double angleMargin) {

    const Vector2 &ball = static_cast<Vector2>(World::getBall()->pos);

    Vector2 behindBallPosition = getPositionBehindBallToPosition(distanceBehindBall, position);
    Vector2 deltaBall = behindBallPosition - ball;


    Vector2 trianglePoint1 = ball;
    Vector2 trianglePoint2 = ball + deltaBall.rotate(M_PI*angleMargin).scale(1.2);
    Vector2 trianglePoint3 = ball + deltaBall.rotate(M_PI*- angleMargin).scale(1.2);

    bool inLargeTriangleOnPosition = control::ControlUtils::pointInTriangle(robotPosition, trianglePoint1, trianglePoint2, trianglePoint3);
    bool closeToLineBehindball= control::ControlUtils::distanceToLineWithEnds(robotPosition, behindBallPosition, ball) < 0.1;

    return inLargeTriangleOnPosition || closeToLineBehindball;
}

Vector2 GeneralPositionCoach::getDemoKeeperGetBallPos(Vector2 ballPos){
    return ballPos+Vector2(0.2,0);
}

} // coach
} // ai
} // rtt