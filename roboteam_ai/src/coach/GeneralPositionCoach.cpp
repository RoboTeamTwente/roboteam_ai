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
#include "GeneralPositionCoach.h"

namespace rtt {
namespace ai {
namespace coach {

// create the global object
GeneralPositionCoach g_generalPositionCoach;

rtt::Vector2 GeneralPositionCoach::getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal) {
    const Vector2 &goal = (ourGoal ? world::field->get_our_goal_center() : world::field->get_their_goal_center());
    return getPositionBehindBallToPosition(distanceBehindBall, goal);
}

Vector2 GeneralPositionCoach::getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot,
        const unsigned int &robotID) {
    Vector2 robot;
    if (world::world->getRobotForId(robotID, ourRobot)) {
        robot = world::world->getRobotForId(robotID, ourRobot).get()->pos;
        return getPositionBehindBallToPosition(distanceBehindBall, robot);
    }
    return Vector2();
}

Vector2 GeneralPositionCoach::getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position) {
    auto ball = world::world->getBall();
    if (! ball) return {};
    Vector2 ballPos = ball->pos;
    return ballPos + (ballPos - position).stretchToLength(distanceBehindBall);
}

Vector2 GeneralPositionCoach::getPositionBehindPositionToPosition(
        double distanceBehindBall, const Vector2 &behindPosition, const Vector2 &toPosition) {
    return behindPosition + (behindPosition - toPosition).stretchToLength(distanceBehindBall);
}

bool GeneralPositionCoach::isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal,
        const Vector2 &robotPosition, double angleMargin) {
    const Vector2 &goal = (ourGoal ? world::field->get_our_goal_center() : world::field->get_their_goal_center());
    return isRobotBehindBallToPosition(distanceBehindBall, goal, robotPosition, angleMargin);
}

bool GeneralPositionCoach::isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot,
        const unsigned int &robotID,
        const Vector2 &robotPosition, double angleMargin) {
    Vector2 robot;
    if (world::world->getRobotForId(robotID, ourRobot)) {
        robot = world::world->getRobotForId(robotID, ourRobot).get()->pos;
        return isRobotBehindBallToPosition(distanceBehindBall, robot, robotPosition, angleMargin);
    }
    return false;
}

bool GeneralPositionCoach::isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position,
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

Vector2 GeneralPositionCoach::getDemoKeeperGetBallPos(Vector2 ballPos) {
    return ballPos + Vector2(0.2, 0);
}
std::vector<Vector2> GeneralPositionCoach::getPenaltyPositions(int number) {

    auto lengthOffset = rtt::ai::world::field->get_field().field_length/4.0;
    auto widthOffset = rtt::ai::world::field->get_field().field_width/4.0;

    std::vector<Vector2> temp = {{- lengthOffset, widthOffset},
                                 {0, widthOffset},
                                 {lengthOffset, widthOffset},
                                 {lengthOffset, - widthOffset},
                                 {0, - widthOffset},
                                 {- lengthOffset, - widthOffset}};

    std::vector<Vector2> res;
    for (int i = 0; i < number; i ++) {
        res.emplace_back(temp.at(i));
    }
    return res;

}
std::vector<Vector2> GeneralPositionCoach::getFreeKickPositions(int number) {
    // Two defenders, one robot to receive the ball, rest 3 in a diagonal
    auto lengthOffset = rtt::ai::world::field->get_field().field_length/4.0;
    auto widthOffset = rtt::ai::world::field->get_field().field_width/4.0;
    Vector2 penaltyUs = rtt::ai::world::field->getPenaltyPoint(true);
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 penaltyThem = rtt::ai::world::field->getPenaltyPoint(false);
    int ballPosMultiplier = (ballPos.y >= 0 ? (- 1) : 1);
    Vector2 lineProgress = {- 0.4, 0};


    Vector2 def1 = {penaltyUs.x + lengthOffset/3.0, penaltyUs.y + widthOffset/1.5};
    Vector2 def2 = {penaltyUs.x + lengthOffset/3.0, - (penaltyUs.y + widthOffset/1.5)};


    Vector2 line1 = {penaltyThem.x - (lengthOffset/3.0), (penaltyThem.y + widthOffset)*ballPosMultiplier};
    Vector2 line2 = line1 + lineProgress;
    Vector2 line3 = line2 + lineProgress;

    std::vector<Vector2> temp = {line1, def1, def2, line2, line3};
    std::vector<Vector2> res;
    for (int i = 0; i < number; i ++) {
        res.emplace_back(temp.at(i));
    }
    return res;
}
std::vector<Vector2> GeneralPositionCoach::getDefendFreeKick(int number) {
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

    std::vector<Vector2> temp = {lineBegin, def1, line3, def2, line2, def3, def4};

    std::vector<Vector2> res;
    for (int i = 0; i < number; i ++) {
        res.emplace_back(temp.at(i));
    }
    return res;
}
std::vector<Vector2> GeneralPositionCoach::getDefendPenaltyPositions(int number) {
    auto lengthOffset = rtt::ai::world::field->get_field().field_length/100.0;
    auto widthOffset = rtt::ai::world::field->get_field().field_width/4.0;
    Vector2 goalUS = rtt::ai::world::field->get_our_goal_center();
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 penaltyUs = rtt::ai::world::field->getPenaltyPoint(true);

    Vector2 lineProgress = {0, 0.4};
    Vector2 lineBegin = {rtt::ai::world::field->getPenaltyPoint(false).x - 0.5, 0};

    Vector2 line2 = lineBegin + lineProgress;
    Vector2 line3 = lineBegin - lineProgress;
    Vector2 line4 = lineBegin - lineProgress*2.0;
    Vector2 line5 = lineBegin + lineProgress*2.0;

    Vector2 def1 = {penaltyUs.x + lengthOffset, penaltyUs.y + widthOffset/2.0};
    Vector2 def2 = {penaltyUs.x + lengthOffset, - (penaltyUs.y + widthOffset/2.0)};

    std::vector<Vector2> temp = {lineBegin, def1, line3, def2, line2, line4, line5};

    std::vector<Vector2> res;
    for (int i = 0; i < number; i ++) {
        res.emplace_back(temp.at(i));
    }
    return res;
}

} // coach
} // ai
} // rtt