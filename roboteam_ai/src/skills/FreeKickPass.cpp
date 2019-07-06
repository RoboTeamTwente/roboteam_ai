//
// Created by robzelluf on 7/2/19.
//

#include "FreeKickPass.h"
#include <roboteam_ai/src/control/ballHandling/BallHandlePosControl.h>

namespace rtt {
namespace ai {

FreeKickPass::FreeKickPass(string name, bt::Blackboard::Ptr blackboard)
        :Pass(std::move(name), std::move(blackboard)) { }

void FreeKickPass::onInitialize() {
    if(properties->hasInt("maxTries")) {
        maxTries = properties->getInt("maxTries");
    } else {
        maxTries = 3;
    }

    robotToPassToID = - 1;
    passInitialized = false;
    hasShot = false;
    fails = 0;
    forcePass = false;
}

void FreeKickPass::makeCommand() {
    auto shotdata = robot->getShotController()->getRobotCommand(*robot, getKicker(), false, control::PASS,
                                                                false, control::LOW);
    command = shotdata.makeROSCommand();
}

}
}
