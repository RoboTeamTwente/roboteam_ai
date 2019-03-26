//
// Created by robzelluf on 12/7/18.
//

#include "DefendOnRobot.h"

namespace rtt{
namespace ai{

std::map<int, int> DefendOnRobot::defencePairs = {};

DefendOnRobot::DefendOnRobot(std::string name, bt::Blackboard::Ptr blackboard)
    :Skill(std::move(name), std::move(blackboard)) { }

void DefendOnRobot::onInitialize() {
    opponentWithBallID = World::whichBotHasBall(false);
    if (opponentWithBallID == -1) {
        currentProgress = FAIL;
    }

    opponentToCoverID = pickOpponentToCover();
    if (opponentToCoverID == -1) {
        currentProgress = FAIL;
    } else {
        defencePairs.insert({opponentToCoverID, robot->id});
    }
}

void DefendOnRobot::onTerminate(Skill::Status s) {
   defencePairs.erase(robot->id);
}


int DefendOnRobot::pickOpponentToCover() {
//    rtt::ai::dangerfinder::DangerData DangerData = dangerfinder::DangerFinder::instance().getMostRecentData();
//    std::vector<int> dangerList = DangerData.dangerList;
//    for (int &opponentID : dangerList) {
//        if (defencePairs.find(opponentID) == defencePairs.end()) {
//            if (! World::theirBotHasBall(static_cast<unsigned int>(opponentID))) {
//                return opponentID;
//            }
//        }
//        else if (defencePairs[opponentID] == robot->id) {
//            return opponentID;
//        }
//    }

    return - 1;
}

bt::Node::Status DefendOnRobot::onUpdate() {
    opponentWithBall = World::getRobotForId(static_cast<unsigned int>(opponentWithBallID), false);
    opponentToCover = World::getRobotForId(static_cast<unsigned int>(opponentToCoverID), false);

    if (opponentWithBall && opponentToCover) {
        updateRobot();
        if (!World::botHasBall(opponentWithBall->id,false)) {
            return Status::Success;
        }

        Vector2 targetPos = calculateLocation();

        std::cout << "Robot:" << robot->id << "TargetPos:" << targetPos << std::endl;
        goToPos.goToPos(robot, targetPos, control::PosControlType::NUMERIC_TREES);

        return Status::Running;
    } else {
        return Status::Failure;
    }
}

Vector2 DefendOnRobot::calculateLocation() {
    float robotAngle1 = opponentWithBall.get()->angle;
    float robotAngle2 = opponentToCover.get()->angle;

    if (opponentWithBall->pos.x > opponentToCover->pos.x) {
        angleBetweenRobots = atan((opponentToCover->pos.y - opponentWithBall->pos.y) /
                                        (opponentWithBall->pos.x - opponentToCover->pos.x));
    } else {
        angleBetweenRobots = ((Vector2)opponentWithBall->pos - opponentToCover->pos).angle();
    }

    double angle1;
    if (robotAngle1 >= 0) {
        angle1 = (angleBetweenRobots - (M_PI - robotAngle1)) / 2;
    } else {
        angle1 = (angleBetweenRobots + (M_PI - robotAngle1)) / 2;
    }
    double angle2 = (-robotAngle2 - angleBetweenRobots) / 2;
    if (robotAngle2 > 0) {
        angle2 += M_PI;
    }

    double distanceBetweenRobots = sqrt(pow(opponentWithBall->pos.x - opponentToCover->pos.x, 2) + pow(opponentToCover->pos.y - opponentWithBall->pos.y, 2));
    double length = distanceBetweenRobots * sin(angle2) / sin(M_PI - angle1 - angle2);

    double xLength = length * cos(angle1);
    double yLength = length * sin(angle1);

    if (opponentWithBall->pos.x > opponentToCover->pos.x) {
        newPosition = {opponentWithBall->pos.x - xLength, opponentWithBall->pos.y + yLength};
    } else {
        newPosition = {opponentWithBall->pos.x - xLength, opponentWithBall->pos.y - yLength};
    }

    return newPosition;
}

}
}
