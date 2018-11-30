//
// Created by mrlukasbos on 30-11-18.
//

#ifndef ROBOTEAM_AI_ROBOT_H
#define ROBOTEAM_AI_ROBOT_H


#include <roboteam_msgs/WorldRobot.h>
#include <QtGui/QColor>

class Robot {
public:
    Robot(roboteam_msgs::WorldRobot robot, bool ourTeam);
    QColor getColor();
private:
    roboteam_msgs::WorldRobot robot;
    bool ourTeam;
};


#endif //ROBOTEAM_AI_ROBOT_H
