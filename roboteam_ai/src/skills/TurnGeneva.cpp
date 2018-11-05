//
// Created by kjhertenberg on 24-10-18.
//

#include "TurnGeneva.h"

namespace rtt {
namespace ai {

void TurnGeneva::Initialize() {
    amountOfCycles = 0;
}

bt::Node::Status TurnGeneva::Update() {

    amountOfCycles ++;
    if (amountOfCycles > MAX_GENEVA_CYCLES) {
        return Status::Failure;
    }

    // Get genevaState from blackboard, otherwise it is a default value.
    int genevaState = properties->hasInt("genevaState") ? properties->getInt("genevaState")
                                                        : DEFAULT_GENEVA_STATE;

    // Send the robotCommand.
    sendGenevaCommand(genevaState);

    return Status::Running;
}

void TurnGeneva::sendGenevaCommand(int genevaState) {
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    // TODO the robot does not kick while turning, do we want to add that to the computer code as well
    command.geneva_state = genevaState;

    publishRobotCommand(command);
}

} // ai
} // rtt