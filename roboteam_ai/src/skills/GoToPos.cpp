//
// Created by baris on 24/10/18.
//
#include "GoToPos.h"
namespace rtt{
namespace ai {


void GoToPos::Initialize() {

    if(blackboard->HasFloat("X") && blackboard->HasInt("Y")) {
        Vector2 posVector(blackboard->GetInt("X"), blackboard->GetInt("Y")); //TODO: look into putting vectors in BB
        targetPos = posVector;
        sendMoveCommand(targetPos);
        currentProgress = Progression::ON_THE_WAY;
    } else {
        ROS_ERROR("No good X and Y set in BB, GoToPos");
        currentProgress = Progression::FAIL;

    }

}

bt::Node::Status GoToPos::Update() {

    // See if the progress is a failure
    if (currentProgress == Progression::FAIL) {
        return status::Failure;
    }

    // Have we already send a command? Should never come to this but it is nice to check
    if (!commandSend) {
        sendMoveCommand(targetPos);
        currentProgress = Progression::ON_THE_WAY;
        return status::Running;
    }

    // Now check the progress we made



}

bool GoToPos::checkTargetPos(Vector2 pos) {
    // TODO: actually check
    return true;
}

void GoToPos::sendMoveCommand(Vector2 pos) {
    if (!checkTargetPos(pos)) {
        ROS_ERROR("Target position is not correct GoToPos");
        return;
    }
    roboteam_msgs::RobotCommand cmd;
    cmd.id = robot.id;
    // TODO: fix this with Control people
    cmd.x_vel = 1;
    cmd.y_vel = 1;
    publishRobotCommand(cmd);
}

GoToPos::Progression GoToPos::checkProgression() {
    // Check the position of the robot
    // Se if it is closer
    // Return some progression ENUM

    return FAIL;
}
} // ai
} // rtt