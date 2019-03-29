//
// Created by kjhertenberg on 24-10-18.
//

#include "TurnGeneva.h"
#include "../utilities/Constants.h"

namespace rtt {
namespace ai {

void TurnGeneva::onInitialize() {
    amountOfCycles = 0;
}

bt::Node::Status TurnGeneva::onUpdate() {

    amountOfCycles ++;
    if (amountOfCycles > Constants::MAX_GENEVA_CYCLES()) {
        return Status::Failure;
    }

    // Get genevaState from blackboard, otherwise it is a default value.
    int genevaState = properties->hasInt("genevaState") ? properties->getInt("genevaState")
                                                        : Constants::DEFAULT_GENEVA_STATE();

    // Send the robotCommand.
    sendGenevaCommand(genevaState);

    return Status::Running;
}

void TurnGeneva::sendGenevaCommand(int genevaState) {
    // TODO the robot does not kick while turning, do we want to add that to the computer code as well
    command.geneva_state = genevaState;

    publishRobotCommand();
}

} // ai
} // rtt