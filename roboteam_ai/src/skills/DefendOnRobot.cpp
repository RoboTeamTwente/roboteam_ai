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
        robot1 = World::getRobotForId(static_cast<unsigned int>(opponentID1), false);
    }

    int opponentID2 = coach::Coach::pickOpponentToCover(robot->id);
    if (opponentID2 == -1) {
        currentProgress = FAIL;
    } else {
        robot2 = World::getRobotForId(static_cast<unsigned int>(opponentID2), false);
        coach::Coach::defencePairs.insert({robot2->id, robot->id});
    }
}

void DefendOnRobot::terminate(Skill::Status s) {
    coach::Coach::defencePairs.erase(robot->id);
}

bt::Node::Status DefendOnRobot::update() {
    if(!coach::Coach::doesRobotHaveBall(robot1->id, false)) {
        return Status::Success;
    }

    Vector2 targetPos = calculateLocation();
    control::ControlGoToPos::goToPos(robot, targetPos, goType::luTh);

    return Status::Running;
}

Vector2 DefendOnRobot::calculateLocation() {
    float robotAngle1 = robot1.get()->angle;
    float robotAngle2 = robot2.get()->angle;

    float angleBetweenRobots = atan((robot2->pos.y - robot1->pos.y) / (robot1->pos.x - robot2->pos.x));

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

    double distanceBetweenRobots = sqrt(pow(robot1->pos.x - robot2->pos.x, 2) + pow(robot2->pos.y - robot1->pos.y, 2));
    double length = distanceBetweenRobots * sin(angle2) / sin(M_PI - angle1 - angle2);

    double xLength = length * cos(angle1);
    double yLength = length * sin(angle1);

    Vector2 newPosition = {robot1->pos.x - xLength, robot1->pos.y + yLength};
    return newPosition;
}

}
}
