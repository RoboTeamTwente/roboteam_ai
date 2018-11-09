#include <utility>

//
// Created by baris on 24/10/18.
//

#include "GoToPos.h"

namespace rtt {
namespace ai {

/// Init the GoToPos skill
void GoToPos::Initialize() {

    if (properties->hasInt("X") && properties->hasInt("Y")) {
        Vector2 posVector(properties->getInt("X"),
                properties->getInt("Y")); //TODO: look into putting vectors in BB
        targetPos = posVector;
        sendMoveCommand(targetPos);
        currentProgress = Progression::ON_THE_WAY;
        robot.id = (unsigned int)RobotDealer::findRobotForRole("testRole");
    }
    else {
        ROS_ERROR("No good X, Y or ROBOT_ID set in BB, GoToPos");
        currentProgress = Progression::FAIL;
    }

}

/// Get an update on the skill
bt::Node::Status GoToPos::Update() {

    // See if the progress is a failure
    if (currentProgress == Progression::FAIL) {
        return status::Failure;
    }

    // Have we already send a command? Should never come to this but it is nice to check
    if (! commandSend) {
        sendMoveCommand(targetPos);
        currentProgress = Progression::ON_THE_WAY;
        return status::Running;
    }

    // Now check the progress we made
    currentProgress = checkProgression();

    switch (currentProgress) {

        // Return the progression in terms of status
    case ON_THE_WAY:return status::Running;
    case DONE:return status::Success;
    case FAIL:return status::Failure;
    }

    return status::Failure;
}

/// Check if the vector is a valid one
bool GoToPos::checkTargetPos(Vector2 pos) {
    // TODO: actually check
    return true;
}

/// Send a move robot command with a vector
void GoToPos::sendMoveCommand(Vector2 pos) {
    if (! checkTargetPos(pos)) {
        ROS_ERROR("Target position is not correct GoToPos");
        return;
    }
    // TODO: get correct kp from 20-sim model
    float proportionalGain = 3; //Proportional gain, kp
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    // TODO: fix this with Control people
    command.x_vel = static_cast<float>(proportionalGain*(robot.pos.x - pos.x));
    command.y_vel = static_cast<float>(proportionalGain*(robot.pos.y - pos.y));
    publishRobotCommand(command);
    commandSend = true;
    std::cout << "                  xvel: " << command.x_vel << ", yvel: " << command.y_vel << std::endl;
}

/// Check the progress the robot made and alter the currentProgress
GoToPos::Progression GoToPos::checkProgression() {
    // Check the position of the robot
    // Se if it is closer
    // Return some progression ENUM
    // TODO: what do we consider there? Should there be an error margin of a cm or so?

    if (targetPos != robot.pos) {
        return ON_THE_WAY;
    }
    else if (targetPos == robot.pos) {
        return DONE;
    }

    return FAIL;
}

GoToPos::GoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}


} // ai
} // rtt