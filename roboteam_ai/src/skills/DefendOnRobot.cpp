//
// Created by robzelluf on 12/7/18.
//

#include "DefendOnRobot.h"

namespace rtt{
namespace ai{

DefendOnRobot::DefendOnRobot(std::string name, bt::Blackboard::Ptr blackboard)
    :Skill(name, blackboard) { }

void DefendOnRobot::initialize() {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        } else {}
        //ROS_ERROR("DefendOnRobot -> robot does not exist in world");
        currentProgress = Progression::FAIL;
        return;
    } else {
        //ROS_ERROR("DefendOnRobot -> ROLE WAITING!!");
        currentProgress = Progression::FAIL;
        return;
    }
    amountOfCycles = 0;
}

bt::Node::Status DefendOnRobot::update() {
    // Fail if we did not succeed after a number of cycles
    amountOfCycles ++;
    if (amountOfCycles > 30) {
        return Status::Failure;
    }

    return Status::Running;
}

Vector2 DefendOnRobot::calculateBestPosition(roboteam_msgs::WorldRobot robot1, roboteam_msgs::WorldRobot robot2) {
    float robotAngle1 = robot1.angle;
    float robotAngle2 = robot2.angle;

    float angleBetweenRobots = tan((robot2.pos.x - robot1.pos.x) / (robot2.pos.y - robot1.pos.y));

    float angle1 = abs(robotAngle1 + angleBetweenRobots) / 2;
    float angle2 = abs(M_PI - robotAngle2 - angleBetweenRobots) / 2;

    float distanceBetweenRobots = sqrt(pow(robot1.pos.x - robot2.pos.x, 2) + pow(robot2.pos.y - robot1.pos.y, 2));
    float x = angle2;
    float y = M_PI - angle1 - angle2;
    float length = distanceBetweenRobots * sin(angle2) / sin(M_PI - angle1 - angle2);

    float xLength = length * cos(angle1);
    float yLength = length * sin(angle1);

    Vector2 newPosition = {robot1.pos.x + xLength, robot1.pos.y + yLength};
    return newPosition;
}
}
}
