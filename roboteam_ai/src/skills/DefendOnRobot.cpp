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
}

bt::Node::Status DefendOnRobot::update() {
    updateRobot();
    return Status::Running;
}

Vector2 DefendOnRobot::calculateBestPosition(roboteam_msgs::WorldRobot robot1, roboteam_msgs::WorldRobot robot2) {
    float robotAngle1 = robot1.angle;
    float robotAngle2 = robot2.angle;

    float angleBetweenRobots = atan((robot2.pos.y - robot1.pos.y) / (robot2.pos.x - robot1.pos.x));

    float angle1 = (angleBetweenRobots - robotAngle1) / 2;
    double angle2 = (M_PI + robotAngle2 - angleBetweenRobots) / 2;

    double distanceBetweenRobots = sqrt(pow(robot1.pos.x - robot2.pos.x, 2) + pow(robot2.pos.y - robot1.pos.y, 2));
    double length = distanceBetweenRobots * sin(angle2) / sin(M_PI - angle1 - angle2);

    double xLength = length * cos(angle1);
    double yLength = length * sin(angle1);

    Vector2 newPosition = {robot1.pos.x + xLength, robot1.pos.y + yLength};
    return newPosition;
}
}
}
