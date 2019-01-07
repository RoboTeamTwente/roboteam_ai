//
// Created by thijs on 10-12-18.
//


#include "ControlGoToPos.h"

namespace rtt {
namespace ai {
namespace control {

void ControlGoToPos::clear(GoToType goToType) {
    switch (goToType) {
    case noPreference:break;
    case ballControl:break;
    case basic:break;
    case lowLevel:break;
    case highLevel:break;
    case force:break;
    case luTh: {
        gtpLuth.clear();
        break;
    }
    case bezier:break;
    }
}

void ControlGoToPos::goToPos(RobotPtr robot, Vector2 &position) {
    GoToType goToType = basic;
    ControlGoToPos::goToPos(std::move(robot), position, goToType);
}

void ControlGoToPos::goToPos(RobotPtr robot, Vector2 &position, GoToType goToType) {

    // TODO: auto switch to low level? maybe
    //    if (distanceToTarget(robot, position) < errorMargin) {
    //        ControlGoToPos::goToPosLowLevel(robot, position);
    //        return;
    //    }


    switch (goToType) {
    case noPreference: {
        ControlGoToPos::goToPos(robot, position);
        break;
    }
    case ballControl: {
        ControlGoToPos::goToPosBallControl(robot, position);
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
void ControlGoToPos::goToPosBallControl(RobotPtr robot, Vector2 &targetPos) {
    Command command = gtpBallControl.goToPos(std::move(robot), targetPos);
    publishRobotCommand(command);
}

void ControlGoToPos::goToPosBasic(RobotPtr robot, Vector2 &targetPos) {

    if (! robot) return;

//    if (! checkTargetPos(targetPos)) {
//        ROS_ERROR("Target position is not correct GoToPos");
//        return;
//    }

    static Controller pidBasic(3, 0.1, 1);
    Vector2 error;
    error.x = targetPos.x - robot->pos.x;
    error.y = targetPos.y - robot->pos.y;
    double dist = error.length();
    static bool far = true;
    if (dist>rtt::ai::constants::ROBOT_RADIUS and !far) {
        pidBasic.setD(1.5);
        far = true;
    }
    else {
        pidBasic.setD(0);
        far = false;
    }
    Vector2 delta = pidBasic.controlPIR2(error, robot->vel);
    Command command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(delta.angle());
    command.x_vel = static_cast<float>(delta.x);
    command.y_vel = static_cast<float>(delta.y);
    publishRobotCommand(command);
}

void ControlGoToPos::goToPosForce(RobotPtr robot, Vector2 &targetPos) {

}

void ControlGoToPos::goToPosLuTh(RobotPtr robot, Vector2 &targetPos) {
    if ((static_cast<Vector2>(robot->pos) - targetPos).length() < 0.50) {
        goToPosBasic(robot, targetPos);
    } else {
        Command command = gtpLuth.goToPos(robot, targetPos);
        publishRobotCommand(command);
    }

}

void ControlGoToPos::goToPosLowLevel(RobotPtr robot, Vector2 &targetPos) {

}
void ControlGoToPos::goToPosHighLevel(RobotPtr robot, Vector2 &targetPos) {

}
void ControlGoToPos::goToPosBezier(RobotPtr robot, Vector2 &targetPos) {

}

void ControlGoToPos::publishRobotCommand(roboteam_msgs::RobotCommand &command) {
    ioManager.publishRobotCommand(command);
}

double ControlGoToPos::distanceToTarget(RobotPtr robot, Vector2 &targetPos) {

    double dx = targetPos.x - robot->pos.x;
    double dy = targetPos.y - robot->pos.y;
    Vector2 deltaPos = {dx, dy};
    return deltaPos.length();
}
ControlGoToPos::ControlGoToPos() {
    rtt::ai::io::IOManager temp(false, true);
    ioManager = temp;
}

} //control
} //ai
} //rtt