//
// Created by thijs on 10-12-18.
//


#include "ControlGoToPos.h"

namespace control {

void ControlGoToPos::goToPos(RobotPtr robot, rtt::Vector2 &position) {
    GoToType goToType = basic;
    ControlGoToPos::goToPos(std::move(robot), position, goToType);
}

void ControlGoToPos::goToPos(RobotPtr robot, rtt::Vector2 &position, GoToType goToType) {
    switch (goToType) {
    case noPreference: {
        break;
    }
    case basic: {
        ControlGoToPos::goToPosBasic(std::move(robot), position);
    }
    case force: {
        ControlGoToPos::goToPosForce(std::move(robot), position);
        break;
    }
    case luTh: {
        ControlGoToPos::goToPosLuTh(std::move(robot), position);
        break;
    }
    case lowLevel: {
        ControlGoToPos::goToPosLowLevel(std::move(robot), position);
        break;
    }
    case highLevel: {
        ControlGoToPos::goToPosHighLevel(std::move(robot), position);
        break;
    }
    case bezier: {
        ControlGoToPos::goToPosBezier(std::move(robot), position);
        break;
    }
    }
}

void ControlGoToPos::goToPosBasic(RobotPtr robot, rtt::Vector2 &targetPos) {

    if (!robot) return;

//    if (! checkTargetPos(targetPos)) {
//        ROS_ERROR("Target position is not correct GoToPos");
//        return;
//    }

    double dx = targetPos.x - robot->pos.x;
    double dy = targetPos.y - robot->pos.y;
    rtt::Vector2 deltaPos = {dx, dy};
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(deltaPos.angle());
    Vector2 deltaPosUnit=deltaPos.normalize();

    command.x_vel = (float) deltaPosUnit.x*2;// abs(angularVel)/(abs(angularVel)-1);
    command.y_vel = (float) deltaPosUnit.y*2;
    publishRobotCommand(command);
}

void ControlGoToPos::goToPosForce(RobotPtr robot, rtt::Vector2 &targetPos) {

}
void ControlGoToPos::goToPosLuTh(RobotPtr robot, rtt::Vector2 &targetPos) {

}
void ControlGoToPos::goToPosLowLevel(RobotPtr robot, rtt::Vector2 &targetPos) {

}
void ControlGoToPos::goToPosHighLevel(RobotPtr robot, rtt::Vector2 &targetPos) {

}
void ControlGoToPos::goToPosBezier(RobotPtr robot, rtt::Vector2 &targetPos) {

}


void ControlGoToPos::publishRobotCommand(roboteam_msgs::RobotCommand &command) {
    rtt::ai::io::IOManager ioManager;
    ioManager.publishRobotCommand(command);
}

} // control