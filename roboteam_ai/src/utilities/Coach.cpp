//
// Created by baris on 6-12-18.
//

#include "Coach.h"
#include "../interface/InterfaceValues.h"
namespace rtt {
namespace ai {
namespace coach {

using dealer = robotDealer::RobotDealer;


int Coach::pickOpponentToCover(int selfID) {
//    dangerfinder::DangerData DangerData = dangerfinder::DangerFinder::instance().getMostRecentData();
//    std::vector<int> dangerList = DangerData.dangerList;
//    for (int &opponentID : dangerList) {
//        if (defencePairs.find(opponentID) == defencePairs.end()) {
//            if (! World::theirBotHasBall(static_cast<unsigned int>(opponentID))) {
//                return opponentID;
//            }
//        }
//        else if (defencePairs[opponentID] == selfID) {
//            return opponentID;
//        }
//    }

    return - 1;
}

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

std::pair<int, bool> Coach::getRobotClosestToBall() {
    auto closestUs = World::getRobotClosestToPoint(World::get_world().us, World::getBall()->pos);
    auto closestThem = World::getRobotClosestToPoint(World::get_world().them, World::getBall()->pos);

    auto distanceToBallUs = (Vector2(closestUs->pos).dist(Vector2(World::getBall()->pos)));
    auto distanceToBallThem = (Vector2(closestThem->pos).dist(Vector2(World::getBall()->pos)));

    roboteam_msgs::WorldRobot closestRobot;
    bool weAreCloser;
    if (distanceToBallUs < distanceToBallThem) {
        closestRobot = * closestUs;
        weAreCloser = true;
    } else {
        closestRobot = * closestThem;
        weAreCloser = false;
    }

    return std::make_pair(closestRobot.id, weAreCloser);
}

std::shared_ptr<roboteam_msgs::WorldRobot> Coach::getRobotClosestToBall(bool isOurTeam) {
    return World::getRobotClosestToPoint(isOurTeam ? World::get_world().us : World::get_world().them, World::getBall()->pos);
}
Vector2 Coach::getDemoKeeperGetBallPos(Vector2 ballPos){
    return ballPos+Vector2(0.2,0);
}
} //control
} //ai
} //rtt
