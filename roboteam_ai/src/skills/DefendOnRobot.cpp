//
// Created by robzelluf on 12/7/18.
//

#include "DefendOnRobot.h"

namespace rtt{
namespace ai{

DefendOnRobot::DefendOnRobot(std::string name, bt::Blackboard::Ptr blackboard)
    :Skill(std::move(name), std::move(blackboard)) { }

void DefendOnRobot::onInitialize() {
    opponentWithBallID = Coach::Coach::whichRobotHasBall(false);
    if (opponentWithBallID == -1) {
        currentProgress = FAIL;
    }

    opponentToCoverID = Coach::Coach::pickOpponentToCover(robot->id);
    if (opponentToCoverID == -1) {
        currentProgress = FAIL;
    } else {
        Coach::Coach::defencePairs.insert({opponentToCoverID, robot->id});
    }
}

void DefendOnRobot::onTerminate(Skill::Status s) {
    Coach::Coach::defencePairs.erase(robot->id);
}

bt::Node::Status DefendOnRobot::onUpdate() {
    opponentWithBall = World::getRobotForId(static_cast<unsigned int>(opponentWithBallID), false);
    opponentToCover = World::getRobotForId(static_cast<unsigned int>(opponentToCoverID), false);

    if (opponentWithBall && opponentToCover) {
        updateRobot();
        if (!Coach::Coach::doesRobotHaveBall(opponentWithBall->id, false)) {
            return Status::Success;
        }

        Vector2 targetPos = calculateLocation();

        std::cout << "Robot:" << robot->id << "TargetPos:" << targetPos << std::endl;
        goToPos.goToPos(robot, targetPos, GoToType::luTh);

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
