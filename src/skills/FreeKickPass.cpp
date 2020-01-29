//
// Created by robzelluf on 7/2/19.
//

#include "skills/FreeKickPass.h"
#include <control/ballHandling/BallHandlePosControl.h>

namespace rtt::ai {

FreeKickPass::FreeKickPass(string name, bt::Blackboard::Ptr blackboard) : Pass(std::move(name), std::move(blackboard)) {}

void FreeKickPass::onInitialize() {
    if (properties->hasInt("maxTries")) {
        maxTries = properties->getInt("maxTries");
    } else {
        maxTries = 3;
    }

    robotToPassToID = -1;
    passInitialized = false;
    hasShot = false;
    fails = 0;
    forcePass = false;
}

void FreeKickPass::makeCommand() {
    auto shotdata = robot->getShotController()->getRobotCommand(*field, *robot, getKicker(), false, control::PASS,
                                                                false, control::LOW,3);
    command = shotdata.makeROSCommand();
}

}  // namespace rtt::ai
