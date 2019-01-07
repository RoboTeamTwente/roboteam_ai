//
// Created by rolf on 14/12/18.
//
// TODO: Test real robot rotation speeds.
// TODO: Make the robot automatically slow down/speed up if the ball is going to one end of the dribbler. Control?
#include "dribbleRotate.h"
namespace rtt {
namespace ai {
DribbleRotate::DribbleRotate(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }
void DribbleRotate::checkProgression() { }
void DribbleRotate::onInitialize() {
    if (properties->hasDouble("Angle")){
        targetAngle=properties->getDouble("Angle");
    }
    else {
        ROS_ERROR(" dribbleRotate Initialize -> No good angle set in properties");
        currentProgression= FAIL;
    }
    startAngle=robot->w;
    incrementAngle=control::ControlUtils::constrainAngle(targetAngle-startAngle);
}
DribbleRotate::Status DribbleRotate::onUpdate() { }
void DribbleRotate::onTerminate(Status s) {
    if (s==Status::Success) {
        roboteam_msgs::RobotCommand command;
        command.id = robot->id;
        command.use_angle = 1;
        command.w = static_cast<float>(targetAngle);
        publishRobotCommand(command);
    }
}
void DribbleRotate::sendMoveCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(targetAngle);
    publishRobotCommand(command);
}
}
}