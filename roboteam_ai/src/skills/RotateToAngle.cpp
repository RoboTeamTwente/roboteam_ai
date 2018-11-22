//
// Created by rolf on 21/11/18.
//

#include "RotateToAngle.h"
namespace rtt{
namespace ai{
RotateToAngle::RotateToAngle(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Return name of the skill
std::string RotateToAngle::node_name() {
    return "RotateToAngle";
}

/// Called when the Skill is Initialized
void RotateToAngle::initialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("RotateToAngle Initialize -> robot does not exist in world");
            currentProgress = Progression::INVALID;
            return;
        }
    }
    else {
        ROS_ERROR("RotateToAngle Initialize -> ROLE INVALID!!");
        currentProgress = Progression::FAIL;
        return;
    }
    if (properties->hasDouble("Angle")) {
        targetAngle = properties->getDouble("Angle");
    }
    else{
        ROS_ERROR("No angle set in properties!");
    }
    if (properties->hasBool("RobotControl")){
        useAngle=properties->getBool("RobotControl");
    }
    else{
        ROS_ERROR("No use_angle identifier set in properties!");
    }

//  ____________________________________________________________________________________________________________________
}

/// Called when the Skill is Updated
RotateToAngle::Status RotateToAngle::update() {

    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    } else {
        ROS_ERROR("RotateToAngle Update -> robot does not exist in world");
        currentProgress = Progression::INVALID;
    }
//  ____________________________________________________________________________________________________________________
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = useAngle;
    command.w=(float)targetAngle; // Angles in grSIM are inverted or something?
    std::cerr << "Rotate command -> id: " << command.id << ", theta: " << command.w << std::endl;
    std::cerr << "Robot Angle: " << robot.angle<<std::endl;
//__________________________________________________________________________________________________________
    deltaAngle=fabs(Control::constrainAngle(targetAngle-robot.angle));
    currentProgress=checkProgression();

    switch (currentProgress) {
    case ROTATING: {publishRobotCommand(command); return Status::Running;}
    case DONE: return Status::Success;
    case FAIL: return Status::Failure;
    case INVALID: return Status::Invalid;
    }

    return Status::Failure;
}

/// Called when the Skill is Terminated
void RotateToAngle::terminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = useAngle;
    command.w = targetAngle;
    publishRobotCommand(command);
}

RotateToAngle::Progression RotateToAngle::checkProgression() {
    double errorMargin = M_PI*0.01;
    if (deltaAngle > errorMargin) return ROTATING;
    else return DONE;
}

}
}