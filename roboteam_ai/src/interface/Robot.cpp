//
// Created by mrlukasbos on 30-11-18.
//

#include "Robot.h"

Robot::Robot(roboteam_msgs::WorldRobot robot, bool ourTeam) : robot(robot), ourTeam(ourTeam) {

}

QColor Robot::getColor() {
    if (ourTeam) {
        return Qt::yellow;
    }
    return Qt::blue;
}

