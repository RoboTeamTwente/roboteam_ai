#include <utility>

//
// Created by baris on 24/10/18.
//

#include "GoToPos.h"

namespace rtt {
namespace ai {

/// Init the GoToPos skill
void GoToPos::Initialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robotID = (unsigned int) RobotDealer::findRobotForRole(roleName);
        if (World::getRobotForId(robotID, true)) {
            robot = World::getRobotForId(robotID, true).get();
        } else {
            ROS_ERROR("GoToPos Initialize -> robot does not exist in world");
            currentProgress = Progression::INVALID;
            return;
        }
    } else {
        ROS_ERROR("GoToPos Initialize -> ROLE INVALID!!");
        currentProgress = Progression::INVALID;
        return;
    }
    bool goToBall;
    if (properties->hasBool("goToBall")) {
        goToBall = properties->getBool("goToBall");
    } else goToBall = false;
    if (goToBall) {
        auto ball = World::getBall();
        targetPos = ball.pos;
    } else {
        if (properties->hasVector2("Position")) {
            Vector2 posVector = properties->getVector2("Position");
            targetPos = posVector;
            currentProgress = Progression::ON_THE_WAY;

        }
        else {
            ROS_ERROR("GoToPos Initialize -> No good X, Y or ROBOT_ID set in BB, GoToPos");
            currentProgress = Progression::FAIL;
        }
    }

}

/// Get an update on the skill
bt::Node::Status GoToPos::Update() {

    if (World::getRobotForId(robotID, true)) {
        robot = World::getRobotForId(robotID, true).get();
    } else {
        ROS_ERROR("GoToPos Update -> robot does not exist in world");
        currentProgress = Progression::INVALID;
    }

    // See if the progress is a failure
    if (currentProgress == Progression::FAIL) {
        return status::Failure;
    } else if (currentProgress == Progression::INVALID) {
        return status::Invalid;
    }

    // Send a move command
    sendMoveCommand();

    // Now check the progress we made
    currentProgress = checkProgression();

    switch (currentProgress) {

        // Return the progression in terms of status
    case ON_THE_WAY:return status::Running;
    case DONE:      return status::Success;
    case FAIL:      return status::Failure;
    case INVALID:   return status::Invalid;
    }

    return status::Failure;
}

/// Check if the vector is a valid one
bool GoToPos::checkTargetPos(Vector2 pos) {
    // TODO: actually check
    return true;
}

/// Send a move robot command with a vector
void GoToPos::sendMoveCommand() {
    if (! checkTargetPos(targetPos)) {
        ROS_ERROR("Target position is not correct GoToPos");
        return;
    }
    // TODO: get correct kp from 20-sim model
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    auto angle = (float) getAngularVelocity();
    command.w = angle;

    command.x_vel = 1;
    command.y_vel = 0;

    publishRobotCommand(command);
    commandSend = true;
    std::cerr << "                  xvel: " << command.x_vel << ", yvel: " << command.y_vel << ", w_vel: " << command.w
              << std::endl;
}

/// Check the progress the robot made and alter the currentProgress
GoToPos::Progression GoToPos::checkProgression() {

    double dx = targetPos.x - robot.pos.x;
    double dy = targetPos.y - robot.pos.y;
    double deltaPos = (dx*dx) + (dy*dy);
    double maxMargin = 0.1;                        // max offset or something.

    if (abs(deltaPos) >= maxMargin) {
        return ON_THE_WAY;
    }
    else
        return DONE;
}

GoToPos::GoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
std::string GoToPos::node_name() {
    return "GoToPos";
}
double GoToPos::getAngularVelocity() {

    auto currentAngle = robot.angle;
    Vector2 deltaPos = {targetPos.x - robot.pos.x, targetPos.y - robot.pos.y};
    auto targetAngle = (float) deltaPos.angle();

    float angleDifference = targetAngle - currentAngle;
    while (angleDifference < 0) angleDifference += 2*M_PI;
    while (angleDifference > 2*M_PI) angleDifference -= 2*M_PI;

    double angularErrorMargin = 0.01;
    if (angleDifference < angularErrorMargin || angleDifference > 2*M_PI - angularErrorMargin) {
        return 0.0;
    }

    if (angleDifference > M_PI) return - MAX_ANGULAR_VELOCITY;
    else return MAX_ANGULAR_VELOCITY;

}

} // ai
} // rtt