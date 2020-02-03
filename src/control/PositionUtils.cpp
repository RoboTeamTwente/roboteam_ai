//
// Created by mrlukasbos on 19-3-19.
//

/*
 *
 * This class contains functionality for retreiving some tactical positions in the game
 *
 */

#include "control/PositionUtils.h"
#include <control/ControlUtils.h>
#include <world/Ball.h>
#include <world/FieldComputations.h>
#include <world/Robot.h>
#include <world/World.h>

namespace rtt::ai::control {

    rtt::Vector2 PositionUtils::getPositionBehindBallToGoal(const Field &field, double distanceBehindBall, bool ourGoal) {
        const Vector2 &goal = (ourGoal ? field.getOurGoalCenter() : field.getTheirGoalCenter());
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
        if (!ball) return {};
        Vector2 ballPos = ball->getPos();
        return ballPos + (ballPos - position).stretchToLength(distanceBehindBall);
    }

    Vector2 PositionUtils::getPositionBehindPositionToPosition(
        double distanceBehindBall, const Vector2 &behindPosition, const Vector2 &toPosition) {
        return behindPosition + (behindPosition - toPosition).stretchToLength(distanceBehindBall);
    }

    bool PositionUtils::isRobotBehindBallToGoal(const Field &field, double distanceBehindBall, bool ourGoal,
                                                const Vector2 &robotPosition, double angleMargin) {
        const Vector2 &goal = (ourGoal ? field.getOurGoalCenter() : field.getTheirGoalCenter());
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

        const Vector2 &ball = static_cast<Vector2>(world::world->getBall()->getPos());
        Vector2 behindBallPosition = getPositionBehindBallToPosition(distanceBehindBall, position);
        Vector2 deltaBall = behindBallPosition - ball;

        Vector2 trianglePoint1 = ball;
        Vector2 trianglePoint2 = ball + deltaBall.rotate(M_PI * angleMargin).scale(2.0);
        Vector2 trianglePoint3 = ball + deltaBall.rotate(M_PI * -angleMargin).scale(2.0);

        bool inLargeTriangleOnPosition = control::ControlUtils::pointInTriangle(robotPosition, trianglePoint1,
                                                                                trianglePoint2, trianglePoint3);

        return inLargeTriangleOnPosition;
    }

    std::vector<Vector2> PositionUtils::getPenaltyPositions(const Field &field, int number) {
        auto lengthOffset = field.getFieldLength() / 4.0;
        auto widthOffset = field.getFieldWidth() / 4.0;

        std::vector<Vector2> temp = {{-lengthOffset, widthOffset},
                                     {0, widthOffset},
                                     {lengthOffset, widthOffset},
                                     {lengthOffset, -widthOffset},
                                     {0, -widthOffset},
                                     {-lengthOffset, -widthOffset},
                                     {0, 0},
                                     {0, 1}};

        std::vector<Vector2> res;
        for (int i = 0; i < number; i++) {
            res.emplace_back(temp.at(i));
        }
        return res;
    }

    std::vector<Vector2> PositionUtils::getFreeKickPositions(const Field &field, int number) {
        // Two availableIDs, one robot to receive the ball, rest 3 in a diagonal
        auto lengthOffset = field.getFieldLength() / 4.0;
        auto widthOffset = field.getFieldWidth() / 4.0;
        Vector2 penaltyUs = FieldComputations::getPenaltyPoint(field, true);
        Vector2 ballPos = rtt::ai::world::world->getBall()->getPos();
        Vector2 penaltyThem = FieldComputations::getPenaltyPoint(field, false);
        int ballPosMultiplier = (ballPos.y >= 0 ? (-1) : 1);
        Vector2 lineProgress = {-0.4, 0};

        Vector2 lineFromBallToTheirPenaltyPoint = penaltyThem - ballPos;
        Vector2 rec1 = ballPos + lineFromBallToTheirPenaltyPoint.rotate(toRadians(20)).stretchToLength(lineFromBallToTheirPenaltyPoint.length() / 2.0);
        Vector2 rec2 = ballPos + lineFromBallToTheirPenaltyPoint.rotate(-toRadians(20)).stretchToLength(lineFromBallToTheirPenaltyPoint.length() / 2.0);

        Vector2 def1 = {penaltyUs.x + lengthOffset / 3.0, penaltyUs.y + widthOffset / 1.5};
        Vector2 def2 = {penaltyUs.x + lengthOffset / 3.0, -(penaltyUs.y + widthOffset / 1.5)};
        Vector2 def3 = {penaltyUs.x, penaltyUs.y + widthOffset / 1.5};

        Vector2 line1 = {penaltyThem.x - (lengthOffset / 3.0), (penaltyThem.y + widthOffset) * ballPosMultiplier};
        Vector2 line2 = line1 + lineProgress;
        Vector2 line3 = line2 + lineProgress;

        std::vector<Vector2> temp = {rec1, rec2, line1, def1, def2, line2, line3, def3};
        std::vector<Vector2> res;
        for (int i = 0; i < number; i++) {
            if (temp.size() > i) {
                res.emplace_back(temp.at(i));
            }
        }
        return res;
    }

    std::vector<Vector2> PositionUtils::getDefendFreeKick(const Field &field, int number) {
        // makes a free kick line
        auto lengthOffset = field.getFieldLength() / 100.0;
        auto widthOffset = field.getFieldWidth() / 4.0;
        Vector2 goalUS = field.getOurGoalCenter();
        Vector2 ballPos = rtt::ai::world::world->getBall()->getPos();
        Vector2 penaltyUs = FieldComputations::getPenaltyPoint(field, true);

        Vector2 lineProgress = ((goalUS - ballPos).stretchToLength(0.28)).rotate(M_PI_2);
        Vector2 lineBegin = ballPos + (goalUS - ballPos).stretchToLength(0.75);

        Vector2 line2 = lineBegin + lineProgress;
        Vector2 line3 = lineBegin - lineProgress;

        Vector2 def1 = {penaltyUs.x + lengthOffset, penaltyUs.y + widthOffset / 2.0};
        Vector2 def2 = {penaltyUs.x + lengthOffset, -(penaltyUs.y + widthOffset / 2.0)};
        Vector2 def3 = def1 + (def2 - def1).stretchToLength((def2 - def1).length() / 3.0);
        Vector2 def4 = (def2 - def3).stretchToLength((def2 - def3).length() / 2.0) + def3;
        Vector2 def5 = {penaltyUs.x + lengthOffset * 1.3, penaltyUs.y};

        std::vector<Vector2> temp = {lineBegin, def1, line3, def2, line2, def3, def4, def5};

        std::vector<Vector2> res;
        for (int i = 0; i < number; i++) {
            res.emplace_back(temp.at(i));
        }
        return res;
    }
    std::vector<Vector2> PositionUtils::getDefendPenaltyPositions(const Field &field, int number) {
        Vector2 lineProgress = {0, 0.4};

        Vector2 lineBegin = {FieldComputations::getPenaltyPoint(field, true).x + 1.05, 0};
        Vector2 line2 = lineBegin + lineProgress;
        Vector2 line3 = lineBegin - lineProgress;
        Vector2 line4 = lineBegin - lineProgress * 2.0;
        Vector2 line5 = lineBegin + lineProgress * 2.0;

        Vector2 atk1 = {0, 0.5};
        Vector2 atk2 = {0, -0.5};
        Vector2 atk3 = {0, 0};

        std::vector<Vector2> temp = {lineBegin, line3, line2, line4, line5, atk1, atk2, atk3};

        std::vector<Vector2> res;
        for (int i = 0; i < number; i++) {
            if (i < temp.size()) {
                res.emplace_back(temp.at(i));
            }
        }
        return res;
    }

}  // namespace rtt::ai::control