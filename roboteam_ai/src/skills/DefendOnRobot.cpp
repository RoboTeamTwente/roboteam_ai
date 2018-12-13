//
// Created by robzelluf on 12/7/18.
//

#include "DefendOnRobot.h"

namespace rtt{
namespace ai{

DefendOnRobot::DefendOnRobot(std::string name, bt::Blackboard::Ptr blackboard)
    :Skill(name, blackboard) { }

void DefendOnRobot::initialize() {
    robot = getRobotFromProperties(properties);
    int opponentID1 = coach::Coach::whichRobotHasBall(false);
    if (opponentID1 == -1) {
        currentProgress = FAIL;
    } else {
        opponentWithBall = World::getRobotForId(static_cast<unsigned int>(opponentID1), false);
    }

    int opponentID2 = coach::Coach::pickOpponentToCover(robot->id);
    if (opponentID2 == -1) {
        currentProgress = FAIL;
    } else {
        opponentToCover = World::getRobotForId(static_cast<unsigned int>(opponentID2), false);
        coach::Coach::defencePairs.insert({opponentToCover->id, robot->id});
    }
}

void DefendOnRobot::terminate(Skill::Status s) {
    coach::Coach::defencePairs.erase(robot->id);
}

bt::Node::Status DefendOnRobot::update() {
    if (opponentWithBall && opponentToCover) {
        updateRobot();
        if (!coach::Coach::doesRobotHaveBall(opponentWithBall->id, false)) {
            return Status::Success;
        }

        Vector2 targetPos = calculateLocation();
        std::cout << targetPos << std::endl;
        goToPos.goToPos(robot, targetPos, goType::basic);

        return Status::Running;
    }
}

Vector2 DefendOnRobot::calculateLocation() {
    float robotAngle1 = opponentWithBall.get()->angle;
    float robotAngle2 = opponentToCover.get()->angle;

    float angleBetweenRobots = atan((opponentToCover->pos.y - opponentWithBall->pos.y) / (opponentWithBall->pos.x - opponentToCover->pos.x));

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

    Vector2 newPosition = {opponentWithBall->pos.x - xLength, opponentWithBall->pos.y + yLength};
    return newPosition;
}

}
}
