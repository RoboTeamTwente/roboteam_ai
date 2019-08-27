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
#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/world/Robot.h>
#include <roboteam_ai/src/world/Ball.h>
#include "include/roboteam_ai/control/PositionUtils.h"

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
std::vector<Vector2> PositionUtils::getPenaltyPositions(int number) {

    auto lengthOffset = rtt::ai::world::field->get_field().field_length/4.0;
    auto widthOffset = rtt::ai::world::field->get_field().field_width/4.0;

    std::vector<Vector2> temp = {{- lengthOffset, widthOffset},
                                 {0, widthOffset},
                                 {lengthOffset, widthOffset},
                                 {lengthOffset, - widthOffset},
                                 {0, - widthOffset},
                                 {- lengthOffset, - widthOffset},
                                 {0,0},
                                 {0, 1}};

    std::vector<Vector2> res;
    for (int i = 0; i < number; i ++) {
        res.emplace_back(temp.at(i));
    }
    return res;

}
std::vector<Vector2> PositionUtils::getFreeKickPositions(int number) {
    // Two availableIDs, one robot to receive the ball, rest 3 in a diagonal
    auto lengthOffset = rtt::ai::world::field->get_field().field_length/4.0;
    auto widthOffset = rtt::ai::world::field->get_field().field_width/4.0;
    Vector2 penaltyUs = rtt::ai::world::field->getPenaltyPoint(true);
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 penaltyThem = rtt::ai::world::field->getPenaltyPoint(false);
    int ballPosMultiplier = (ballPos.y >= 0 ? (- 1) : 1);
    Vector2 lineProgress = {- 0.4, 0};


    Vector2 lineFromBallToTheirPenaltyPoint = penaltyThem - ballPos;
    Vector2 rec1 = ballPos + lineFromBallToTheirPenaltyPoint.rotate(toRadians(20)).stretchToLength(lineFromBallToTheirPenaltyPoint.length()/2.0);
    Vector2 rec2 = ballPos + lineFromBallToTheirPenaltyPoint.rotate(-toRadians(20)).stretchToLength(lineFromBallToTheirPenaltyPoint.length()/2.0);

    Vector2 def1 = {penaltyUs.x + lengthOffset/3.0, penaltyUs.y + widthOffset/1.5};
    Vector2 def2 = {penaltyUs.x + lengthOffset/3.0, - (penaltyUs.y + widthOffset/1.5)};
    Vector2 def3 = {penaltyUs.x, penaltyUs.y + widthOffset/1.5};

    Vector2 line1 = {penaltyThem.x - (lengthOffset/3.0), (penaltyThem.y + widthOffset)*ballPosMultiplier};
    Vector2 line2 = line1 + lineProgress;
    Vector2 line3 = line2 + lineProgress;

    std::vector<Vector2> temp = {rec1, rec2, line1, def1, def2, line2, line3, def3};
    std::vector<Vector2> res;
    for (int i = 0; i < number; i ++) {
        if (temp.size() > i) {
            res.emplace_back(temp.at(i));
        }
    }
    return res;
}
std::vector<Vector2> PositionUtils::getDefendFreeKick(int number) {
    // makes a free kick line
    auto lengthOffset = rtt::ai::world::field->get_field().field_length/100.0;
    auto widthOffset = rtt::ai::world::field->get_field().field_width/4.0;
    Vector2 goalUS = rtt::ai::world::field->get_our_goal_center();
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 penaltyUs = rtt::ai::world::field->getPenaltyPoint(true);

    Vector2 lineProgress = ((goalUS-ballPos).stretchToLength(0.28)).rotate(M_PI_2);
    Vector2 lineBegin = ballPos + (goalUS - ballPos).stretchToLength(0.75);

    Vector2 line2 = lineBegin + lineProgress;
    Vector2 line3 = lineBegin - lineProgress;

    Vector2 def1 = {penaltyUs.x + lengthOffset, penaltyUs.y + widthOffset/2.0};
    Vector2 def2 = {penaltyUs.x + lengthOffset, - (penaltyUs.y + widthOffset/2.0)};
    Vector2 def3 = def1 + (def2-def1).stretchToLength((def2-def1).length()/3.0);
    Vector2 def4 = (def2-def3).stretchToLength((def2-def3).length()/2.0) + def3;
    Vector2 def5 = {penaltyUs.x + lengthOffset*1.3, penaltyUs.y};

    std::vector<Vector2> temp = {lineBegin, def1, line3, def2, line2, def3, def4, def5};

    std::vector<Vector2> res;
    for (int i = 0; i < number; i ++) {
        res.emplace_back(temp.at(i));
    }
    return res;
}
std::vector<Vector2> PositionUtils::getDefendPenaltyPositions(int number) {
    Vector2 lineProgress = {0, 0.4};

    Vector2 lineBegin = {rtt::ai::world::field->getPenaltyPoint(true).x + 1.05, 0};
    Vector2 line2 = lineBegin + lineProgress;
    Vector2 line3 = lineBegin - lineProgress;
    Vector2 line4 = lineBegin - lineProgress*2.0;
    Vector2 line5 = lineBegin + lineProgress*2.0;

    Vector2 atk1 = {0, 0.5};
    Vector2 atk2 = {0, -0.5};
    Vector2 atk3 = {0, 0};

    std::vector<Vector2> temp = {lineBegin, line3, line2, line4, line5, atk1, atk2, atk3};

    std::vector<Vector2> res;
    for (int i = 0; i < number; i ++) {
        if (i < temp.size()) {
            res.emplace_back(temp.at(i));
        }
    }
    return res;
}


} // coach
} // ai
} // rtt